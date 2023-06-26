/* UDP MultiCast Send/Receive Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"


#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "driver/i2s_std.h"
#include "driver/gpio.h"

#define MULTICAST_IPV4_ADDR	"239.69.83.134"
#define UDP_PORT		5004

#define MULTICAST_LOOPBACK	CONFIG_EXAMPLE_LOOPBACK
#define MULTICAST_TTL		CONFIG_EXAMPLE_MULTICAST_TTL

#define SDP_IPV4_ADDR		"239.255.255.255"
#define	SDP_PORT		9875

#define	SDP_RECIEVE_ENTRY_MAX	(16)

#define BUTTON_GPIO		(2)
//#define BUTTON_GPIO		(0)		// do not use GPIO0 if you use I2S2,because of bug

#define RTP_HEADER_SIZE                 (12)
//#define EXAMPLE_BUFF_SIZE               ((16/8)*2*48*1)
#define EXAMPLE_BUFF_SIZE               ((16/8)*2*48*5)


static const char *TAG = "multicast";
static const char *V4TAG = "mcast-ipv4";

static int reload_request          = 0;
static int source_cur              = 0;

static int socket_add_ipv4_multicast_group(int sock, bool assign_source_if, char *ipv4_addr, int udp)
{
    struct ip_mreq imreq = { 0 };
    struct in_addr iaddr = { 0 };
    int err = 0;
    // Configure source interface
    imreq.imr_interface.s_addr = IPADDR_ANY;
    // Configure multicast address to listen to
    err = inet_aton(ipv4_addr, &imreq.imr_multiaddr.s_addr);
    if (err != 1) {
        ESP_LOGE(V4TAG, "Configured IPV4 multicast address '%s' is invalid.", ipv4_addr);
        // Errors in the return value have to be negative
        err = -1;
        goto err;
    }
    ESP_LOGI(TAG, "Configured IPV4 Multicast address %s", inet_ntoa(imreq.imr_multiaddr.s_addr));
    if (!IP_MULTICAST(ntohl(imreq.imr_multiaddr.s_addr))) {
        ESP_LOGW(V4TAG, "Configured IPV4 multicast address '%s' is not a valid multicast address. This will probably not work.", ipv4_addr);
    }

    if (assign_source_if) {
        // Assign the IPv4 multicast source interface, via its IP
        // (only necessary if this socket is IPV4 only)
        err = setsockopt(sock, IPPROTO_IP, IP_MULTICAST_IF, &iaddr,
                         sizeof(struct in_addr));
        if (err < 0) {
            ESP_LOGE(V4TAG, "Failed to set IP_MULTICAST_IF. Error %d", errno);
            goto err;
        }
    }

    err = setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                         &imreq, sizeof(struct ip_mreq));
    if (err < 0) {
        ESP_LOGE(V4TAG, "Failed to set IP_ADD_MEMBERSHIP. Error %d", errno);
        goto err;
    }

 err:
    return err;
}

static int create_multicast_dst_ipv4_socket(char *ipv4_addr, int udp)
{
    struct sockaddr_in saddr = { 0 };
    int sock = -1;
    int err = 0;

    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(V4TAG, "Failed to create socket. Error %d", errno);
        return -1;
    }

    // Bind the socket to any address
    saddr.sin_family = PF_INET;
    saddr.sin_port = htons(udp);
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    err = bind(sock, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in));
    if (err < 0) {
        ESP_LOGE(V4TAG, "Failed to bind socket. Error %d", errno);
        goto err;
    }


    // Assign multicast TTL (set separately from normal interface TTL)
    uint8_t ttl = MULTICAST_TTL;
    setsockopt(sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(uint8_t));
    if (err < 0) {
        ESP_LOGE(V4TAG, "Failed to set IP_MULTICAST_TTL. Error %d", errno);
        goto err;
    }

#if MULTICAST_LOOPBACK
    // select whether multicast traffic should be received by this device, too
    // (if setsockopt() is not called, the default is no)
    uint8_t loopback_val = MULTICAST_LOOPBACK;
    err = setsockopt(sock, IPPROTO_IP, IP_MULTICAST_LOOP,
                     &loopback_val, sizeof(uint8_t));
    if (err < 0) {
        ESP_LOGE(V4TAG, "Failed to set IP_MULTICAST_LOOP. Error %d", errno);
        goto err;
    }
#endif

    // this is also a listening socket, so add it to the multicast
    // group for listening...
    err = socket_add_ipv4_multicast_group(sock, true, ipv4_addr, udp);
    if (err < 0) {
        goto err;
    }

    // All set, socket is configured for sending and receiving
    return sock;

err:
    close(sock);
    return -1;
}

static void mcast_example_task(void *pvParameters)
{
    int sock, i, tmp1, rc;
    unsigned char sendbuf[2000];
    //char raddr_name[32] = { 0 };
    int seq_no = 0, len, err;
    struct sockaddr_storage raddr; // Large enough for IPv4
    socklen_t socklen = sizeof(raddr);
    fd_set rfds, rfds_default;
    i2s_chan_handle_t rx_handle;
    size_t bytes_written;
    int pcm_msec = 5;		// 1 or 5 msec interval
    //int rtp_payload_size;	// 204 (L16 1ms) or 300 (L24 1ms) or 972 (L16 5ms) or 1452 (L24 5ms)
    int pcm_byte_per_frame = 4;	// 8 (L24) or 4 (L16)

    //rtp_payload_size = pcm_byte_per_frame * 48 * pcm_msec;


    while (1) {
        struct timeval tv = {
            .tv_sec = 0,
            .tv_usec = 50 * 1000,	// 50 msec
        };
        char addrbuf[32] = { 0 };

        struct addrinfo hints = {
            .ai_flags = AI_PASSIVE,
            .ai_socktype = SOCK_DGRAM,
        };
        struct addrinfo *res;

        sock = create_multicast_dst_ipv4_socket(MULTICAST_IPV4_ADDR, UDP_PORT);
        if (sock < 0) {
            ESP_LOGE(TAG, "Failed to create IPv4 multicast socket");
        }

        if (sock < 0) {
            // Nothing to do!
            vTaskDelay(5 / portTICK_PERIOD_MS);
            continue;
        }

        // set destination multicast addresses for sending from these sockets
        struct sockaddr_in sdestv4 = {
            .sin_family = PF_INET,
            .sin_port = htons(UDP_PORT),
        };
        // We know this inet_aton will pass because we did it above already
        inet_aton(MULTICAST_IPV4_ADDR, &sdestv4.sin_addr.s_addr);

        FD_ZERO(&rfds_default);
        FD_SET(sock, &rfds_default);

//ESP_LOGI(TAG,"s_addr=%lX", (((struct sockaddr_in *)&raddr)->sin_addr.s_addr));
        if ( 1 ) {
                hints.ai_family = AF_INET; // For an IPv4 socket

                int err = getaddrinfo(MULTICAST_IPV4_ADDR, NULL, &hints, &res);
                if (err < 0) {
                    ESP_LOGE(TAG, "getaddrinfo() failed for IPV4 destination address. error: %d", err);
                    break;
                }
                if (res == 0) {
                    ESP_LOGE(TAG, "getaddrinfo() did not return any addresses");
                    break;
                }
                ((struct sockaddr_in *)res->ai_addr)->sin_port = htons(UDP_PORT);
                inet_ntoa_r(((struct sockaddr_in *)res->ai_addr)->sin_addr, addrbuf, sizeof(addrbuf)-1);
                ESP_LOGI(TAG, "Sending to IPV4 multicast address %s:%d...",  addrbuf, UDP_PORT);

/*
            switch (len) {
                case 204:  pcm_byte_per_frame = 4; pcm_msec = 1;  // L16 1ms
                    break;
                case 300:  pcm_byte_per_frame = 8; pcm_msec = 1;  // L24 1ms
                    break;
                case 972:  pcm_byte_per_frame = 4; pcm_msec = 5;  // L16 5ms
                    break;
                case 1452: pcm_byte_per_frame = 8; pcm_msec = 5;  // L24 5ms
                    break;
            }
*/

            i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
            //i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
            ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));


            i2s_std_config_t std32_cfg = {
                .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
                .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    //            .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
                .gpio_cfg = {
//                    .mclk = I2S_GPIO_UNUSED,
                    .mclk = GPIO_NUM_0,
                    .bclk = GPIO_NUM_26,
                    .ws = GPIO_NUM_22,
                    .dout = I2S_GPIO_UNUSED,
                    .din = GPIO_NUM_25,
                    .invert_flags = {
                        .mclk_inv = false,
                        .bclk_inv = false,
                        .ws_inv = false,
                    },
                },
            };

            /* Initialize the channel */
            ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std32_cfg));

            /* Before write data, start the rx channel first */
            ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
        }


        // Loop waiting for UDP received, and sending UDP packets if we don't
        // see any.
        err = 1;

        ESP_LOGI(TAG, "start loop");
        while (1) {
            size_t r_bytes = 0;

            // Nothing to do!
            //vTaskDelay(0);
            vTaskDelay(5 / portTICK_PERIOD_MS);
            if (i2s_channel_read(rx_handle, sendbuf+RTP_HEADER_SIZE, EXAMPLE_BUFF_SIZE, &r_bytes, 1000) == ESP_OK) {
                //ESP_LOGI(TAG, "i2s_channel_read(),len=%d", r_bytes);
                for (i=0; i<pcm_byte_per_frame*48*pcm_msec; i+=pcm_byte_per_frame>>1) {
                    tmp1          = sendbuf[12+i];
                    sendbuf[12+i] = sendbuf[13+i];
                    sendbuf[13+i] = tmp1;
                }
            } else {
                ESP_LOGE(TAG, "Read Task: i2s read failed");
            }
            sendbuf[2] = (seq_no & 0xff00) >> 8;
            sendbuf[3] = (seq_no & 0xff);
            if (!gpio_get_level(BUTTON_GPIO)) {
                for (i=0; i<pcm_byte_per_frame*48*pcm_msec; i+=pcm_byte_per_frame>>1) {
                    if (i >= 24*5) {
                        sendbuf[12+i] = 0x00;
                        sendbuf[13+i] = 0x00;
                    } else {
                        sendbuf[12+i] = 0x08;
                        sendbuf[13+i] = 0x00;
                    }
                }
            }
            err = sendto(sock, sendbuf, EXAMPLE_BUFF_SIZE+RTP_HEADER_SIZE, 0, res->ai_addr, res->ai_addrlen);
            //ESP_LOGI(TAG, "seq_no=%d, sendto()=%d", seq_no, err);
            ++seq_no;
        }

        /* Have to stop the channel before deleting it */
        i2s_channel_disable(rx_handle);
        /* If the handle is not needed any more, delete it to release the channel resources */
        i2s_del_channel(rx_handle);

        ESP_LOGE(TAG, "Shutting down socket and restarting...");
        freeaddrinfo(res);
        shutdown(sock, 0);
        close(sock);

    }

}

static void manage_example_task(void *pvParameters)
{
    int sock, i, rc, dst_ipv4_port;
    char sendbuf[384], sdp_buf[128], *p, if_addr[20];
    int sdp_len;
    char p1 = 0, p2[64];
    long src_ipv4_addr_long;
    //char raddr_name[32] = { 0 };
    int len, err;
    struct sockaddr_storage raddr; // Large enough for IPv4
    socklen_t socklen = sizeof(raddr);
    fd_set rfds, rfds_default;

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(get_example_netif(),&ip_info);
    strcpy(if_addr, inet_ntoa(ip_info.ip.addr));


    esp_rom_gpio_pad_select_gpio(BUTTON_GPIO);
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);  //PULLUP が必要  

    //rtp_payload_size = pcm_byte_per_frame * 48 * pcm_msec;

    source_cur = 0;

    sendbuf[0] = 0x20;     // Flags
    sendbuf[1] = 0x00;     // Authenticattion Length
    sendbuf[2] = 0x12;     // Message Identifier Hash
    sendbuf[3] = 0x34;     // Message Identifier Hash
    strncpy( &sendbuf[8], "application/sdp\0", 16);
    sprintf(sdp_buf, "v=0\r\no=- 1 0 IN IP4 %s\r\ns=Keio\r\nc=IN IP4 %s/32\r\nm=audio %d RTP/AVP\na=rtpmap:97 L16/48000/2\r\n", if_addr, MULTICAST_IPV4_ADDR, UDP_PORT); //if_addr, ip_addr, port);
    sdp_len = strlen( sdp_buf );
    strcpy( &sendbuf[24], sdp_buf);

    while (1) {
        struct timeval tv = {
            .tv_sec = 0,
            .tv_usec = 50 * 1000,	// 50 msec
        };
        char addrbuf[32] = { 0 };
        struct addrinfo hints = {
            .ai_flags = AI_PASSIVE,
            .ai_socktype = SOCK_DGRAM,
        };
        struct addrinfo *res;

        sock = create_multicast_dst_ipv4_socket(SDP_IPV4_ADDR, SDP_PORT);
        if (sock < 0) {
            ESP_LOGE(TAG, "Failed to create IPv4 multicast socket");
        }

        if (sock < 0) {
                // Nothing to do!
                vTaskDelay(5 / portTICK_PERIOD_MS);
                continue;
        }

        // set destination multicast addresses for sending from these sockets
        struct sockaddr_in sdestv4 = {
            .sin_family = PF_INET,
            .sin_port = htons(SDP_PORT),
        };
        // We know this inet_aton will pass because we did it above already
        inet_aton(SDP_IPV4_ADDR, &sdestv4.sin_addr.s_addr);

        FD_ZERO(&rfds_default);
        FD_SET(sock, &rfds_default);


        hints.ai_family = AF_INET; // For an IPv4 socket

        int err = getaddrinfo(SDP_IPV4_ADDR, NULL, &hints, &res);
        if (err < 0) {
            ESP_LOGE(TAG, "getaddrinfo() failed for IPV4 destination address. error: %d", err);
            break;
        }
        if (res == 0) {
            ESP_LOGE(TAG, "getaddrinfo() did not return any addresses");
            break;
        }
        ((struct sockaddr_in *)res->ai_addr)->sin_port = htons(SDP_PORT);
        inet_ntoa_r(((struct sockaddr_in *)res->ai_addr)->sin_addr, addrbuf, sizeof(addrbuf)-1);

        vTaskDelay(5000 / portTICK_PERIOD_MS);

        while (1) {

            ESP_LOGI(TAG, "SDP: %s", sdp_buf);
            err = sendto(sock, sendbuf, 24 + sdp_len, 0, res->ai_addr, res->ai_addrlen);
            if (err < 0) {
                ESP_LOGE(TAG, "Select failed: errno %d", errno);
            }

            vTaskDelay(30000 / portTICK_PERIOD_MS);

        }

        ESP_LOGE(TAG, "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
    }

}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    //xTaskCreate(&mcast_example_task, "mcast_task", 4096, NULL, 5, NULL);
    xTaskCreatePinnedToCore(&mcast_example_task, "mcast_task", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(&manage_example_task, "manage_task", 4096, NULL, 5, NULL, 1);

}
