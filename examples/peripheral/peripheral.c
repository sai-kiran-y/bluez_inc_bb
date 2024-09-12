#include <glib.h>
#include <stdio.h>
#include <signal.h>
#include <pthread.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include "adapter.h"
#include "device.h"
#include "logger.h"
#include "agent.h"
#include "application.h"
#include "advertisement.h"
#include "utility.h"
#include "parser.h"
#include <termios.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/time.h>  // Add this line
#include <stdint.h>


#define TAG "Main"

#define VEHICLE_SERVICE_UUID "0000fff0-0000-1000-8000-00805f9b34fb"
#define CAN_CHAR_UUID "0000fff1-0000-1000-8000-00805f9b34fb"
#define GPS_CHAR_UUID "0000fff2-0000-1000-8000-00805f9b34fb"

#define GPS_FREQ_CHAR_UUID "0000fff3-0000-1000-8000-00805f9b34fb"
#define CAN_FREQ_CHAR_UUID "0000fff4-0000-1000-8000-00805f9b34fb"
#define IMU_FREQ_CHAR_UUID "0000fff5-0000-1000-8000-00805f9b34fb"

#define UNLOCK_VEHICLE_CHAR_UUID "0000fff6-0000-1000-8000-00805f9b34fb"
#define TCU_INFO_CHAR_UUID "0000fff7-0000-1000-8000-00805f9b34fb"

#define AUTH_SERVICE_UUID "0000a000-0000-1000-8000-00805f9b34fb"
#define PASSWORD_CHAR_UUID "0000a001-0000-1000-8000-00805f9b34fb"
#define IS_AUTHENTICATED_CHAR_UUID "0000a002-0000-1000-8000-00805f9b34fb"

#define CUD_CHAR "00002901-0000-1000-8000-00805f9b34fb"

#define DEFAULT_PASSWORD 0x123456
#define DEFAULT_PASSWORD_LEN 3

#define BLUEZ_ERROR_AUTHORIZATION_FAILED "org.bluez.Error.Failed"

#define NUM_CAN_IDS 11
#define CAN_FRAME_SIZE sizeof(struct can_frame)
#define TIMESTAMP_SIZE sizeof(struct timeval)
#define CAN_DATA_LEN (NUM_CAN_IDS * (CAN_FRAME_SIZE + TIMESTAMP_SIZE + sizeof(canid_t)))
#define TCU_INFO_MAX_LENGTH 18 // IMEI (15) + comma (1) + device ID (1) + null terminator (1)


// Customizable interval for writing CAN data to characteristic (in seconds)
#define DEFAULT_WRITE_INTERVAL 100

#define AT_COMMAND "AT+GSN\r"
#define DEVICE_PORT "/dev/ttyUSB0"
#define BUFFER_SIZE 256

#define IMEI_LENGTH 15
#define TO_MILLIS 0.001

#define DEVICE_ID "1" // Define device_id as "1" for now

// Global buffer for TCU info
char tcu_info[TCU_INFO_MAX_LENGTH] = {0}; 
static pthread_mutex_t tcu_info_mutex = PTHREAD_MUTEX_INITIALIZER;

char imei[IMEI_LENGTH + 1] = {0};
int tty_fd = -1;
GMainLoop *loop = NULL;
Adapter *default_adapter = NULL;
Advertisement *advertisement = NULL;
Application *app = NULL;
static gboolean is_authenticated = FALSE;
Device *connected_device = NULL;
const canid_t monitored_can_ids[NUM_CAN_IDS] = {
    0x407, 0x520, 0x201, 0x306, 0x303, 0x305, 0x302, 0x322, 0x307, 0x100, 0x500
};


static pthread_mutex_t can_data_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint8_t can_data[CAN_DATA_LEN];  // Buffer containing CAN ID, timestamp, and CAN data
//static struct timeval can_data_timestamp;  // Single timestamp for the entire collection of CAN IDs
static struct can_frame ble_can_id_arr[NUM_CAN_IDS];  // Array to store CAN frames without individual timestamps


static int write_interval = (DEFAULT_WRITE_INTERVAL) * (TO_MILLIS);

unsigned long long current_time_millis() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (unsigned long long)(tv.tv_sec) * 1000 + (tv.tv_usec / 1000);
}

uint64_t toLittleEndian64(uint64_t value) {
    return ((value & 0xFF00000000000000) >> 56) |
           ((value & 0x00FF000000000000) >> 40) |
           ((value & 0x0000FF0000000000) >> 24) |
           ((value & 0x000000FF00000000) >> 8) |
           ((value & 0x00000000FF000000) << 8) |
           ((value & 0x0000000000FF0000) << 24) |
           ((value & 0x000000000000FF00) << 40) |
           ((value & 0x00000000000000FF) << 56);
}

uint32_t toLittleEndian32(uint32_t value) {
    return ((value & 0xFF000000) >> 24) |
           ((value & 0x00FF0000) >> 8) |
           ((value & 0x0000FF00) << 8) |
           ((value & 0x000000FF) << 24);
}


void ble_install_vehicle_service()
{
    log_info(TAG, "Adding Vehicle Service\r\n");

    binc_application_add_service(app, VEHICLE_SERVICE_UUID);

    binc_application_add_characteristic(
            app,
            VEHICLE_SERVICE_UUID,
            CAN_CHAR_UUID,
            GATT_CHR_PROP_READ | GATT_CHR_PROP_NOTIFY); // Support reading and notifying
    
    // Skip GPS for now
    /*
    binc_application_add_characteristic(
            app,
            VEHICLE_SERVICE_UUID,
            GPS_CHAR_UUID,
            GATT_CHR_PROP_NOTIFY);
    
    binc_application_add_characteristic(
            app,
            VEHICLE_SERVICE_UUID,
            GPS_FREQ_CHAR_UUID,
            GATT_CHR_PROP_WRITE);
    */
    binc_application_add_characteristic(
            app,
            VEHICLE_SERVICE_UUID,
            CAN_FREQ_CHAR_UUID,
            GATT_CHR_PROP_WRITE);
    
    binc_application_add_characteristic(
            app,
            VEHICLE_SERVICE_UUID,
            IMU_FREQ_CHAR_UUID,
            GATT_CHR_PROP_WRITE);
    
    binc_application_add_characteristic(
            app,
            VEHICLE_SERVICE_UUID,
            UNLOCK_VEHICLE_CHAR_UUID,
            GATT_CHR_PROP_WRITE);

    binc_application_add_characteristic(
            app,
            VEHICLE_SERVICE_UUID,
            TCU_INFO_CHAR_UUID,
            GATT_CHR_PROP_READ | GATT_CHR_PROP_NOTIFY);
}

void ble_install_auth_service()
{
    log_info(TAG, "Adding Auth Service\r\n");

    binc_application_add_service(app, AUTH_SERVICE_UUID);

    binc_application_add_characteristic(
            app,
            AUTH_SERVICE_UUID,
            PASSWORD_CHAR_UUID,
            GATT_CHR_PROP_WRITE);
    
    binc_application_add_characteristic(
            app,
            AUTH_SERVICE_UUID,
            IS_AUTHENTICATED_CHAR_UUID,
            GATT_CHR_PROP_READ | GATT_CHR_PROP_NOTIFY);
}

void on_powered_state_changed(Adapter *adapter, gboolean state) {
    sleep(2);
    log_debug(TAG, "powered '%s' (%s)", state ? "on" : "off", binc_adapter_get_path(adapter));
}

void publish_is_authenticated() {
    const char *value = is_authenticated ? "true" : "false";
    GByteArray *byteArray = g_byte_array_new();
    g_byte_array_append(byteArray, (const guint8 *)value, strlen(value));
	safe_binc_application_notify(app, AUTH_SERVICE_UUID, IS_AUTHENTICATED_CHAR_UUID, byteArray);
    g_byte_array_unref(byteArray);
}

// Timer callback to publish is_authenticated every second
gboolean publish_is_authenticated_periodically(gpointer user_data) {
    publish_is_authenticated();
    return TRUE; // Returning TRUE ensures the function is called repeatedly
}

void publish_tcu_info() {
    pthread_mutex_lock(&tcu_info_mutex);
	// Convert tcu_info to a byte array for publication
    GByteArray *byteArray = g_byte_array_new();
	if (byteArray == NULL) {
        log_error(TAG, "Failed to allocate memory for GByteArray.");
        pthread_mutex_unlock(&tcu_info_mutex);
        return;
    }
    g_byte_array_append(byteArray, (const guint8 *)tcu_info, strlen(tcu_info));
    safe_binc_application_notify(app, VEHICLE_SERVICE_UUID, TCU_INFO_CHAR_UUID, byteArray);
    //log_debug(TAG, "Published TCU info: %s", tcu_info);
	g_byte_array_unref(byteArray); // Ensure the byte array is properly freed
    pthread_mutex_unlock(&tcu_info_mutex);
}

gboolean publish_tcu_info_periodically(gpointer user_data) {
    publish_tcu_info();
    return TRUE; // Keeps the periodic function running
}

void on_central_state_changed(Adapter *adapter, Device *device) {
    char *deviceToString = binc_device_to_string(device);
    log_debug(TAG, deviceToString);
    g_free(deviceToString);
    connected_device = device;

    log_debug(TAG, "remote central %s is %s", binc_device_get_address(device), binc_device_get_connection_state_name(device));
    ConnectionState state = binc_device_get_connection_state(device);
    if (state == BINC_CONNECTED) {
        binc_adapter_stop_advertising(adapter, advertisement);
		is_authenticated = FALSE;
    } else if (state == BINC_DISCONNECTED){
        binc_adapter_start_advertising(adapter, advertisement);
    }
}

const char *on_local_char_read(const Application *application, const char *address, const char *service_uuid,
                        const char *char_uuid) {

    log_debug(TAG, "on char read");

    if (g_str_equal(service_uuid, AUTH_SERVICE_UUID) && g_str_equal(char_uuid, IS_AUTHENTICATED_CHAR_UUID)) {
        const char *value = is_authenticated ? "true" : "false";
        GByteArray *byteArray = g_byte_array_new();
        log_debug(TAG, "calling g_byte_array_append");
        g_byte_array_append(byteArray, (const guint8 *)value, strlen(value));
        log_debug(TAG, "calling binc_application_set_char_value");
        binc_application_set_char_value(application, service_uuid, char_uuid, byteArray);
        return NULL;
    }

    if (!is_authenticated) {
        log_info(TAG, "Read request rejected: Authentication required");
        return BLUEZ_ERROR_AUTHORIZATION_FAILED;
    }

    if (g_str_equal(service_uuid, VEHICLE_SERVICE_UUID) && g_str_equal(char_uuid, CAN_CHAR_UUID)) {
        pthread_mutex_lock(&can_data_mutex);
        GByteArray *byteArray = g_byte_array_new();
        g_byte_array_append(byteArray, can_data, CAN_DATA_LEN);
        pthread_mutex_unlock(&can_data_mutex);

        log_debug(TAG, "Returning CAN data for read request");
        binc_application_set_char_value(application, service_uuid, char_uuid, byteArray);
        return NULL;
    }

    if (g_str_equal(service_uuid, VEHICLE_SERVICE_UUID) && g_str_equal(char_uuid, TCU_INFO_CHAR_UUID)) {
        pthread_mutex_lock(&can_data_mutex);
        GByteArray *byteArray = g_byte_array_new();
        g_byte_array_append(byteArray, (const guint8 *)imei, strlen(imei));
        pthread_mutex_unlock(&can_data_mutex);

        log_debug(TAG, "Returning IMEI data for read request");
        binc_application_set_char_value(application, service_uuid, char_uuid, byteArray);
        return NULL;
    }

    return BLUEZ_ERROR_REJECTED;
}

const char *on_local_char_write(const Application *application, const char *address, const char *service_uuid,
                                const char *char_uuid, GByteArray *byteArray) {

    log_debug(TAG, "on char write");

    if (g_str_equal(service_uuid, AUTH_SERVICE_UUID) && g_str_equal(char_uuid, PASSWORD_CHAR_UUID)) {
        log_debug(TAG, "Password write received, length: %d", byteArray->len);
        
        if (byteArray->len != DEFAULT_PASSWORD_LEN) {
            log_error(TAG, "Invalid password length: %d (expected %d)", byteArray->len, DEFAULT_PASSWORD_LEN);
            binc_device_disconnect(connected_device);
            return BLUEZ_ERROR_INVALID_VALUE_LENGTH;
        }

        // Manually construct the received password from the byte array
        uint32_t received_password = 0;
        received_password |= byteArray->data[0];
        received_password <<= 8;
        received_password |= byteArray->data[1];
        received_password <<= 8;
        received_password |= byteArray->data[2];

        log_debug(TAG, "Received password: 0x%06x", received_password);

        if (received_password == DEFAULT_PASSWORD) {
            is_authenticated = TRUE;

            // Write "true" to IS_AUTHENTICATED_CHAR_UUID
            //const uint8_t yes_value[] = {'t', 'r', 'u', 'e'};
            //GByteArray *yesArray = g_byte_array_new();
            //g_byte_array_append(yesArray, yes_value, sizeof(yes_value));
            //binc_application_set_char_value(application, AUTH_SERVICE_UUID, IS_AUTHENTICATED_CHAR_UUID, yesArray);
            //g_byte_array_unref(yesArray, TRUE);

            log_info(TAG, "Authentication successful, 'yes' written to IS_AUTHENTICATED_CHAR_UUID");

            // Make VEHICLE_SERVICE_UUID visible
            //ble_install_vehicle_service();
        } else {
            log_error(TAG, "Authentication failed, received password: 0x%06x", received_password);
            // Disconnect the device
            is_authenticated = FALSE;
            binc_device_disconnect(connected_device);
            return BLUEZ_ERROR_AUTHORIZATION_FAILED;
        }
    }

    if (!is_authenticated) {
        log_info(TAG, "Write request rejected: Authentication required");
        return BLUEZ_ERROR_AUTHORIZATION_FAILED;
    }

    if (g_str_equal(service_uuid, VEHICLE_SERVICE_UUID) && g_str_equal(char_uuid, CAN_FREQ_CHAR_UUID)) {
        if (byteArray->len == 1) {
            uint8_t received_interval = byteArray->data[0];
            log_info(TAG, "Received CAN frequency: %u seconds", received_interval);
            write_interval = received_interval * TO_MILLIS;
        } else {
            log_error(TAG, "Invalid CAN frequency length: %d", byteArray->len);
        }
    }

    if (g_str_equal(service_uuid, VEHICLE_SERVICE_UUID) && g_str_equal(char_uuid, UNLOCK_VEHICLE_CHAR_UUID)) {
        if (byteArray->len == 1 && byteArray->data[0] == 0x01) {
            log_info(TAG, "Vehicle unlock command received");
        } else {
            log_info(TAG, "Invalid command");
        }
    }

    return NULL;
}

void on_local_char_start_notify(const Application *application, const char *service_uuid, const char *char_uuid) {
    log_debug(TAG, "on start notify");
}

void on_local_char_stop_notify(const Application *application, const char *service_uuid, const char *char_uuid) {
    log_debug(TAG, "on stop notify");
}

gboolean callback(gpointer data) {
    if (app != NULL) {
        binc_adapter_unregister_application(default_adapter, app);
        binc_application_free(app);
        app = NULL;
    }

    if (advertisement != NULL) {
        binc_adapter_stop_advertising(default_adapter, advertisement);
        binc_advertisement_free(advertisement);
    }

    if (default_adapter != NULL) {
        binc_adapter_free(default_adapter);
        default_adapter = NULL;
    }

    g_main_loop_quit((GMainLoop *) data);
    return FALSE;
}

static void cleanup_handler(int signo) {
    if (signo == SIGINT) {
        log_error(TAG, "received SIGINT, performing graceful shutdown");

        if (app != NULL) {
            binc_adapter_unregister_application(default_adapter, app);
            binc_application_free(app);
            app = NULL;
        }

        if (advertisement != NULL) {
            binc_adapter_stop_advertising(default_adapter, advertisement);
            binc_advertisement_free(advertisement);
        }

        if (default_adapter != NULL) {
            binc_adapter_free(default_adapter);
            default_adapter = NULL;
        }

        if (tty_fd != -1) {
            close(tty_fd);
            tty_fd = -1;
        }

        g_main_loop_quit(loop);
    }
}

void *can_read_thread(void *arg) {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket");
        return NULL;
    }

    // Enable timestamping
    int timestamp_on = 1;
    setsockopt(s, SOL_SOCKET, SO_TIMESTAMP, &timestamp_on, sizeof(timestamp_on));

    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        close(s);
        return NULL;
    }

    struct can_frame frame;
    struct msghdr msg;
    struct iovec iov;
    struct {
        struct cmsghdr cm;
        struct timeval tv;
    } control;

    char ctrlmsg[CMSG_SPACE(sizeof(struct timeval))];
    memset(&msg, 0, sizeof(msg));
    msg.msg_name = &addr;
    msg.msg_namelen = sizeof(addr);
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = &control;
    msg.msg_controllen = sizeof(ctrlmsg);

    while (1) {
        iov.iov_base = &frame;
        iov.iov_len = sizeof(struct can_frame);
        int nbytes = recvmsg(s, &msg, 0);
        if (nbytes < 0) {
            perror("Read");
            break;
        }

        struct timeval tv;
        for (struct cmsghdr *cmsg = CMSG_FIRSTHDR(&msg); cmsg; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
            if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP) {
                memcpy(&tv, CMSG_DATA(cmsg), sizeof(tv));
            }
        }

        gboolean is_monitored = FALSE;
        for (int i = 0; i < NUM_CAN_IDS; i++) {
            if (frame.can_id == monitored_can_ids[i]) {
                is_monitored = TRUE;
                break;
            }
        }

        if (is_monitored) {
            pthread_mutex_lock(&can_data_mutex);

            // Update the specific CAN ID's frame in the array
            for (int i = 0; i < NUM_CAN_IDS; i++) {
                if (frame.can_id == monitored_can_ids[i]) {
                    ble_can_id_arr[i] = frame;
                    break;
                }
            }

            // Clear the global can_data buffer
            memset(can_data, 0, CAN_DATA_LEN);

            // Get the current timestamp and convert it to little-endian format
            unsigned long long timestamp = current_time_millis();
            timestamp = toLittleEndian64(timestamp);

            // Store timestamp at the beginning of the global can_data buffer
            memcpy(can_data, &timestamp, sizeof(timestamp));

            // Adjust the starting position in the buffer to account for the timestamp
            uint8_t *current_buffer = can_data + sizeof(timestamp);

            // Update the global can_data buffer with CAN IDs and their corresponding data
            for (int i = 0; i < NUM_CAN_IDS; i++) {
                // Convert CAN ID to little-endian format
                uint32_t canId = ble_can_id_arr[i].can_id;
                canId = toLittleEndian32(canId);

                // Copy CAN ID to the buffer
                memcpy(current_buffer, &canId, sizeof(canId));

                // Copy data to the buffer
                memcpy(current_buffer + sizeof(canId), ble_can_id_arr[i].data, 8);

                // Move to the next CAN frame position in the buffer
                current_buffer += sizeof(canId) + 8;
            }

            pthread_mutex_unlock(&can_data_mutex);
        }
    }

    close(s);
    return NULL;
}

// Thread for writing CAN data to characteristic
gboolean send_can_data_periodically(gpointer user_data) {
    is_authenticated = TRUE;
        sleep(write_interval);
        
        if (is_authenticated) {
            pthread_mutex_lock(&can_data_mutex);
            GByteArray *byteArray = g_byte_array_new();

            // First, append the timestamp for the collection
            //g_byte_array_append(byteArray, (const guint8 *)&can_data_timestamp, TIMESTAMP_SIZE);

			// Append CAN data
            g_byte_array_append(byteArray, can_data, CAN_DATA_LEN);

            pthread_mutex_unlock(&can_data_mutex);
            //sleep(2); 
            //log_debug(TAG, "Writing CAN data to characteristic");
            safe_binc_application_notify(app, VEHICLE_SERVICE_UUID, CAN_CHAR_UUID, byteArray);
            g_byte_array_unref(byteArray);
        }
        return TRUE;
}

gboolean print_can_data_periodically(gpointer user_data){

    // Print CAN data in the format of "candump can0"
    uint8_t *current_buffer = can_data + sizeof(unsigned long long);  // Skip timestamp part
    for (int i = 0; i < NUM_CAN_IDS; i++) {
        // Read CAN ID from buffer
        uint32_t canId;
        memcpy(&canId, current_buffer, sizeof(canId));
        canId = toLittleEndian32(canId);  // Convert to little-endian format

        // Read CAN data from buffer
        uint8_t data[8];
        memcpy(data, current_buffer + sizeof(canId), 8);

        // Print in "candump can0" format
        printf("can0  %03X  [8] ", canId);
        for (int j = 0; j < 8; j++) {
            printf("%02X ", data[j]);
        }
        printf("\n");

        // Move to the next CAN frame position in the buffer
        current_buffer += sizeof(canId) + 8;
    }
    printf("DONE\n");
    return TRUE;
}

void *read_imei_thread(void *arg) {
    tty_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (tty_fd == -1) {
        log_error(TAG, "Failed to open /dev/ttyUSB0");
        return NULL;
    }

    struct termios options;
    tcgetattr(tty_fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(tty_fd, TCSANOW, &options);

    char at_cmd[] = "AT+GSN\r";
    char buffer[256];
	ssize_t n;
    while (1) {
        tcflush(tty_fd, TCIFLUSH);
        write(tty_fd, at_cmd, strlen(at_cmd));
        memset(buffer, 0, sizeof(buffer));
        usleep(100000);  // Small delay to allow the device to respond

        // Read the response until "OK" is detected
        while ((n = read(tty_fd, buffer, sizeof(buffer) - 1)) > 0) {
            buffer[n] = '\0';  // Null-terminate the string

            // Check if "OK" is part of the response, indicating end of response
            if (strstr(buffer, "OK")) {
                // Find the start of the IMEI in the buffer
                char *imei_start = strchr(buffer, '\n');
                if (imei_start != NULL) {
                    imei_start++; // Move past the newline character
                    pthread_mutex_lock(&tcu_info_mutex);
                    strncpy(imei, imei_start, IMEI_LENGTH);  // Copy only the IMEI digits
                    imei[IMEI_LENGTH] = '\0';  // Ensure null termination
			    	snprintf(tcu_info, sizeof(tcu_info), "%s,%s", imei, DEVICE_ID);
                    pthread_mutex_unlock(&tcu_info_mutex);
                    //log_info(TAG, "IMEI: %s", imei);
                }
                break;  // Break the read loop once "OK" is found
            }
        }

        if (n <= 0) {
            log_error(TAG, "Failed to read full response from AT+GSN command");
        }

		/*
        int n = read(tty_fd, buffer, sizeof(buffer) - 1);
        if (n > 0) {
            buffer[n] = '\0';
            char *ok_pos = strstr(buffer, "OK");
            if (ok_pos) {
                pthread_mutex_lock(&can_data_mutex);
                strncpy(imei, buffer, IMEI_LENGTH);  // IMEI is 15 digits
                imei[IMEI_LENGTH] = '\0';
			    snprintf(tcu_info, sizeof(tcu_info), "%s,%s", imei, DEVICE_ID);
                pthread_mutex_unlock(&can_data_mutex);
                log_info(TAG, "TCU_INFO: %s", tcu_info);
            } else if (strstr(buffer, "ERROR")) {
                log_error(TAG, "ERROR: Could not determine IMEI");
            }
        }
		*/
        usleep(50000);
    }

    close(tty_fd);
    tty_fd = -1;
    return NULL;
}

// Function to read dmesg and detect "hardware error"
gboolean check_dmesg_for_errors(gpointer userdata) {
    FILE *fp;
    char buffer[256];

    // Open the dmesg command for reading
    fp = popen("dmesg | tail -n 1", "r");
    if (fp == NULL) {
        log_error(TAG, "Failed to run dmesg command");
        sleep(1); // Sleep to avoid tight loop
    }

    // Read the output a line at a time
    while (fgets(buffer, sizeof(buffer) - 1, fp) != NULL) {
        if (strstr(buffer, "hardware error") || strstr(buffer, "failed: -110")) {
            log_error(TAG, "Detected hardware error.");
            pclose(fp);

            // Call the shell script to handle hardware error and restart
            system("/home/root/restart_app.sh");
            return TRUE; // Exit thread after handling
        }
    }
    // Close the dmesg command pipe
    pclose(fp);
    return TRUE;
}


int main(void) {

    log_set_level(LOG_DEBUG);

    // Initialize can_data buffer
    memset(can_data, 0, CAN_DATA_LEN);

    // Get a DBus connection
    GDBusConnection *dbusConnection = g_bus_get_sync(G_BUS_TYPE_SYSTEM, NULL, NULL);

    // Setup handler for CTRL+C
    if (signal(SIGINT, cleanup_handler) == SIG_ERR)
        log_error(TAG, "can't catch SIGINT");

    // Setup mainloop
    loop = g_main_loop_new(NULL, FALSE);

    // Get the default adapter
    default_adapter = binc_adapter_get_default(dbusConnection);

    if (default_adapter != NULL) {
        log_debug(TAG, "using adapter '%s'", binc_adapter_get_path(default_adapter));

        // Make sure the adapter is on
        binc_adapter_set_powered_state_cb(default_adapter, &on_powered_state_changed);
        if (!binc_adapter_get_powered_state(default_adapter)) {
            binc_adapter_power_on(default_adapter);
        }

        // Setup remote central connection state callback
        binc_adapter_set_remote_central_cb(default_adapter, &on_central_state_changed);

        // Setup advertisement
        GPtrArray *adv_service_uuids = g_ptr_array_new();
        g_ptr_array_add(adv_service_uuids, VEHICLE_SERVICE_UUID);
        g_ptr_array_add(adv_service_uuids, AUTH_SERVICE_UUID);

        advertisement = binc_advertisement_create();
        binc_advertisement_set_local_name(advertisement, "LxG_iWave_1");
        binc_advertisement_set_services(advertisement, adv_service_uuids);
        g_ptr_array_free(adv_service_uuids, TRUE);
        binc_adapter_start_advertising(default_adapter, advertisement);

        // Start application
        app = binc_create_application(default_adapter);

        // Install services
        ble_install_auth_service();
        ble_install_vehicle_service();

        binc_application_set_char_read_cb(app, &on_local_char_read);
        binc_application_set_char_write_cb(app, &on_local_char_write);
        binc_application_set_char_start_notify_cb(app, &on_local_char_start_notify);
        binc_application_set_char_stop_notify_cb(app, &on_local_char_stop_notify);
        binc_adapter_register_application(default_adapter, app);

        // Create CAN read thread
        pthread_t can_read_tid;
        pthread_create(&can_read_tid, NULL, can_read_thread, NULL);

        // Create IMEI read thread
        pthread_t imei_read_tid;
        pthread_create(&imei_read_tid, NULL, read_imei_thread, NULL);
    } else {
        log_debug("MAIN", "No adapter found");
    }

    // Bail out after some time
    //g_timeout_add_seconds(600, callback, loop);

	// Start the timer to publish is_authenticated every 1 second
	g_timeout_add_seconds(1, publish_is_authenticated_periodically, NULL);

	// Start the timer to publish tcu_info every 1 second
	g_timeout_add_seconds(1, publish_tcu_info_periodically, NULL);

    // Set up periodic CAN data transmission every 3 seconds
    g_timeout_add_seconds(1, send_can_data_periodically, NULL);

    // Set up periodic CAN data transmission every 3 seconds
    g_timeout_add_seconds(5, print_can_data_periodically, NULL);

    // Start the mainloop
    g_main_loop_run(loop);

    // Clean up mainloop
    g_main_loop_unref(loop);

    // Disconnect from DBus
    g_dbus_connection_close_sync(dbusConnection, NULL, NULL);
    g_object_unref(dbusConnection);

    return 0;
}

