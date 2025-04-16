#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <drivers/drv_canfd.h>

#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

extern "C" __EXPORT int canfd_status_main(int argc, char *argv[]);

int canfd_status_main(int argc, char *argv[])
{
	int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	if (sock < 0) {
		PX4_ERR("CAN socket open failed");
		return -1;
	}

	struct ifreq ifr {};
	strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);

	if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
		PX4_ERR("SIOCGIFINDEX failed");
		close(sock);
		return -1;
	}

	// Bitrate is not available through socket, but you can log it manually if known
	printf("CAN FD Interface: can0\n");
	printf("Nominal Bitrate: 500000 (example)\n");
	printf("Data Bitrate: 2000000 (example)\n");

	// Loopback status
	int loopback = 0;
	socklen_t optlen = sizeof(loopback);
	getsockopt(sock, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, &optlen);
	printf("Loopback mode: %s\n", loopback ? "ENABLED" : "DISABLED");

	int recv_own_msgs = 0;
	getsockopt(sock, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, &optlen);
	printf("Receive own messages: %s\n", recv_own_msgs ? "YES" : "NO");

	close(sock);
	return 0;
}
