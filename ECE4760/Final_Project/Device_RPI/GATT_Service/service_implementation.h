/*

	V. Hunter Adams (vha3@cornell.edu)
	Custom GATT Service implementation
	Modeled off examples from BTstack

*/

#include "btstack_defines.h"
#include "ble/att_db.h"
#include "ble/att_server.h"
#include "btstack_util.h"
#include "bluetooth_gatt.h"
#include "btstack_debug.h"

#include "server_demo_gattfile.h"
#include "Protothreads/pt_cornell_rp2040_v1_4.h"

// Create a struct for managing this service
typedef struct {

	// Connection handle for service
	hci_con_handle_t con_handle;

	// Characteristic A information
	float 		characteristic_a_value;
	uint16_t 	characteristic_a_client_configuration;
	char * 		characteristic_a_user_description;

	// Characteristic A handles
	uint16_t  	characteristic_a_handle;
	uint16_t 	characteristic_a_client_configuration_handle;
	uint16_t 	characteristic_a_user_description_handle;

	// Callback functions
	btstack_context_callback_registration_t callback_a;
	btstack_context_callback_registration_t callback_b;
	btstack_context_callback_registration_t callback_c;
	btstack_context_callback_registration_t callback_d;

} custom_service_t;

// Create a callback registration object, and an att service handler object
static att_service_handler_t 	service_handler;
static custom_service_t 		service_object;

// Characteristic user descriptions (appear in LightBlue app)
char characteristic_a[] = "Read-only Counter";
char characteristic_b[] = "DDS Frequency";
char characteristic_c[] = "String from Pico";
char characteristic_d[] = "String to Pico";
char characteristic_e[] = "LED Status and Control";
char characteristic_f[] = "Color Selection";

// Protothreads semaphore
semaphore_t BLUETOOTH_READY;

// Callback functions for ATT notifications on characteristics
static void characteristic_a_callback(void * context)
{
	// Associate the void pointer input with our custom service object
	custom_service_t * instance = (custom_service_t *) context;
	// Send a notification
	att_server_notify(instance->con_handle, instance->characteristic_a_handle, (uint8_t *) &instance->characteristic_a_value, sizeof(instance->characteristic_a_value));
}

// Read callback (no client configuration handles on characteristics without Notify)
static uint16_t custom_service_read_callback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size)
{
	UNUSED(con_handle);

	// Characteristic A
	if (attribute_handle == service_object.characteristic_a_handle)
	{
		return att_read_callback_handle_blob((uint8_t *) &service_object.characteristic_a_value, sizeof(service_object.characteristic_a_value), offset, buffer, buffer_size);
	}
	if (attribute_handle == service_object.characteristic_a_client_configuration_handle)
	{
		return att_read_callback_handle_little_endian_16(service_object.characteristic_a_client_configuration, offset, buffer, buffer_size);
	}
	if (attribute_handle == service_object.characteristic_a_user_description_handle) 
	{
		return att_read_callback_handle_blob(service_object.characteristic_a_user_description, strlen(service_object.characteristic_a_user_description), offset, buffer, buffer_size);
	}

	return 0;
}

// Write callback
static int custom_service_write_callback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size)
{
	UNUSED(transaction_mode);
	UNUSED(offset);
	UNUSED(buffer_size);

	// Enable/disable notifications
	if (attribute_handle == service_object.characteristic_a_client_configuration_handle)
	{
		service_object.characteristic_a_client_configuration = little_endian_read_16(buffer, 0);
		service_object.con_handle = con_handle;
	}

	return 0;
}

/////////////////////////////////////////////////////////////////////////////
////////////////////////////// USER API /////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

// Initialize our custom service handler
void custom_service_server_init(void)
{
	// Initialize the semaphore
	sem_init(&BLUETOOTH_READY, 0, 1);

	// Pointer to our service object
	custom_service_t * instance = &service_object;

	// Assign characteristic value
	instance->characteristic_a_value = 0;

	// Assign characteristic user description
	instance->characteristic_a_user_description = characteristic_a;

	// Assign handle values (from generated gatt header)
	instance->characteristic_a_handle = ATT_CHARACTERISTIC_0000FF11_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE;
	instance->characteristic_a_client_configuration_handle = ATT_CHARACTERISTIC_0000FF11_0000_1000_8000_00805F9B34FB_01_CLIENT_CONFIGURATION_HANDLE;
	instance->characteristic_a_user_description_handle = ATT_CHARACTERISTIC_0000FF11_0000_1000_8000_00805F9B34FB_01_USER_DESCRIPTION_HANDLE;

	// Service start and end handles (modeled off heartrate example)
	service_handler.start_handle = 0;
	service_handler.end_handle = 0xFFFF;
	service_handler.read_callback = &custom_service_read_callback;
	service_handler.write_callback = &custom_service_write_callback;

	// Register the service handler
	att_server_register_service_handler(&service_handler);
}

// Update Characteristic A value
void set_characteristic_a_value(float value)
{
	// Pointer to our service object
	custom_service_t * instance = &service_object;

	// Update field value
	instance->characteristic_a_value = value;

	// Are notifications enabled? If so, register a callback
	if (instance->characteristic_a_client_configuration)
	{
		instance->callback_a.callback = &characteristic_a_callback;
		instance->callback_a.context  = (void*) instance;
		if (att_server_can_send_packet_now(instance->con_handle))
		{
			att_server_register_can_send_now_callback(&instance->callback_a, instance->con_handle);
			// static int counter = 0;
			// printf("%d: Sending angle update\n", counter++);
		}
	}
}