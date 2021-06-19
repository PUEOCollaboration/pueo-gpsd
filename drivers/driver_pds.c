/*
 * Qualcomm PDS Interface driver.
 *
 * Tested in Dragonboard410c (APQ8016) PDS service.
 *
 * This file is Copyright 2020 by Linaro Limited
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  /* must be before all includes */

#include <sys/socket.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include "../include/gpsd.h"

#if defined(PDS_ENABLE)
#include "../include/driver_pds.h"

#include <linux/qrtr.h>

#define QMI_PDS_MAX		16
#define QMI_PDS_SERVICE_ID	0x10
#define QMI_PDS_VERSION		0x2
#define QMI_PDS_PATH_STARTS	6

struct qmi_header {
	uint8_t type;
	uint16_t txn;
	uint16_t msg;
	uint16_t len;
} __attribute__((__packed__));

struct qmi_tlv {
	uint8_t key;
	uint16_t len;
	uint8_t value[];
} __attribute__((__packed__));

static struct gps_device_t *pds_devices[QMI_PDS_MAX];

#define QMI_REQUEST			0
#define QMI_INDICATION			4

#define QMI_LOC_REG_EVENTS		0x21
#define QMI_TLV_EVENT_MASK		1
#define QMI_EVENT_MASK_NMEA		4

#define QMI_LOC_START			0x22
#define QMI_LOC_STOP			0x23
#define QMI_TLV_SESSION_ID		1

#define QMI_LOC_EVENT_NMEA		0x26
#define QMI_TLV_NMEA			1

static ssize_t qmi_pds_connect(struct gps_device_t *session)
{
	struct sockaddr_qrtr sq;
	socklen_t sl = sizeof(sq);
	struct qrtr_ctrl_pkt pkt;
	char *hostname;
	char *endptr;
	int ret;

	session->lexer.outbuflen = 0;

	hostname = session->gpsdata.dev.path + QMI_PDS_PATH_STARTS;
	if (!strcmp(hostname, "any")) {
		session->driver.pds.hostid = -1;
	} else {
		session->driver.pds.hostid = (int)strtol(hostname, &endptr, 10);
		if (endptr == hostname) {
			GPSD_LOG(LOG_ERROR, &session->context->errout,
				 "QRTR open: Invalid node id.\n");
			return -1;
		}
	}

	ret = recvfrom(session->gpsdata.gps_fd, &pkt, sizeof(pkt), 0,
		       (struct sockaddr *)&sq, &sl);
	if (ret < 0) {
		GPSD_LOG(LOG_ERROR, &session->context->errout,
			 "QRTR connect: Unable to receive lookup request.\n");
		return -1;
	}

	if (sl != sizeof(sq) || sq.sq_port != QRTR_PORT_CTRL) {
		GPSD_LOG(LOG_INFO, &session->context->errout,
			 "QRTR connect: Received message is not ctrl message, ignoring.\n");
		return 1;
	}

	if (pkt.cmd != QRTR_TYPE_NEW_SERVER)
		return 1;

	/* All fields zero indicates end of lookup response */
	if (!pkt.server.service && !pkt.server.instance &&
	    !pkt.server.node && !pkt.server.port) {
		GPSD_LOG(LOG_ERROR, &session->context->errout,
			 "QRTR connect: End of lookup, No PDS service found for %s.\n",
			 session->gpsdata.dev.path);
		return -1;
	}

	/* Filter results based on specified node */
	if (session->driver.pds.hostid != -1 &&
			session->driver.pds.hostid != (int)pkt.server.node)
		return 1;

	session->driver.pds.pds_node = pkt.server.node;
	session->driver.pds.pds_port = pkt.server.port;

	GPSD_LOG(LOG_INF, &session->context->errout,
		 "QRTR open: Found PDS at %d %d.\n",
		 session->driver.pds.pds_node,
		 session->driver.pds.pds_port);

	sq.sq_family = AF_QIPCRTR;
	sq.sq_node = session->driver.pds.pds_node;
	sq.sq_port = session->driver.pds.pds_port;
	ret = connect(session->gpsdata.gps_fd, (struct sockaddr *)&sq, sizeof(sq));
	if (ret < 0) {
		GPSD_LOG(LOG_ERROR, &session->context->errout,
			 "QRTR connect: Failed to connect socket to PDS Service.\n");
		return -1;
	}

	session->driver.pds.ready = 1;
	session->device_type->event_hook(session, event_reactivate);
	return 1;
}

static ssize_t qmi_pds_get_packet(struct gps_device_t *session)
{
	struct sockaddr_qrtr sq;
	socklen_t sl = sizeof(sq);
	struct qmi_header *hdr;
	struct qmi_tlv *tlv;
	size_t buflen = sizeof(session->lexer.inbuffer);
	size_t offset;
	void *buf = session->lexer.inbuffer;
	int ret;

	ret = recvfrom(session->gpsdata.gps_fd, buf, buflen, 0,
		       (struct sockaddr *)&sq, &sl);
	if (ret < 0 && errno == EAGAIN) {
		session->lexer.outbuflen = 0;
		return 1;
	} else if (ret < 0) {
		GPSD_LOG(LOG_ERROR, &session->context->errout,
			 "QRTR get: Unable to receive packet.\n");
		return -1;
	}

	/* TODO: Validate sq to be our peer */

	hdr = buf;
	if (hdr->type != QMI_INDICATION ||
	    hdr->msg != QMI_LOC_EVENT_NMEA) {
		session->lexer.outbuflen = 0;
		return ret;
	}

	offset = sizeof(*hdr);
	while (offset < (size_t)ret) {
		tlv = (struct qmi_tlv *)((char*)buf + offset);

		if (offset + sizeof(*tlv) + tlv->len > (size_t)ret)
			break;

		if (tlv->key == QMI_TLV_NMEA) {
			memcpy(session->lexer.outbuffer, tlv->value, tlv->len);
			session->lexer.type = NMEA_PACKET;
			session->lexer.outbuffer[tlv->len] = 0;
			session->lexer.outbuflen = tlv->len;
			break;
		}

		offset += tlv->len;
	}

	return ret;
}

static ssize_t qmi_pds_get(struct gps_device_t *session)
{
	if (!session->driver.pds.ready)
		return qmi_pds_connect(session);
	else
		return qmi_pds_get_packet(session);
}

static void qmi_pds_event_hook(struct gps_device_t *session, event_t event)
{
	struct qmi_header *hdr;
	struct qmi_tlv *tlv;
	static int txn_id;
	char buf[128];
	char *ptr;
	int sock = session->gpsdata.gps_fd;
	int ret;

	switch (event) {
	case event_deactivate:
		if (!session->driver.pds.ready)
			return;

		ptr = buf;
		hdr = (struct qmi_header *)ptr;
		hdr->type = QMI_REQUEST;
		hdr->txn = txn_id++;
		hdr->msg = QMI_LOC_STOP;
		hdr->len = sizeof(*tlv) + sizeof(uint8_t);
		ptr += sizeof(*hdr);

		tlv = (struct qmi_tlv *)ptr;
		tlv->key = QMI_TLV_SESSION_ID;
		tlv->len = sizeof(uint8_t);
		*(uint8_t*)tlv->value = 1;
		ptr += sizeof(*tlv) + sizeof(uint8_t);

		ret = send(sock, buf, ptr - buf, 0);
		if (ret < 0) {
			GPSD_LOG(LOG_ERROR, &session->context->errout,
				 "QRTR event_hook: failed to send STOP request.\n");
			return;
		}
		break;
	case event_reactivate:
		if (!session->driver.pds.ready)
			return;

		ptr = buf;
		hdr = (struct qmi_header *)ptr;
		hdr->type = QMI_REQUEST;
		hdr->txn = txn_id++;
		hdr->msg = QMI_LOC_REG_EVENTS;
		hdr->len = sizeof(*tlv) + sizeof(uint64_t);
		ptr += sizeof(*hdr);

		tlv = (struct qmi_tlv *)ptr;
		tlv->key = QMI_TLV_EVENT_MASK;
		tlv->len = sizeof(uint64_t);
		*(uint64_t*)tlv->value = QMI_EVENT_MASK_NMEA;
		ptr += sizeof(*tlv) + sizeof(uint64_t);

		ret = send(sock, buf, ptr - buf, 0);
		if (ret < 0) {
			GPSD_LOG(LOG_ERROR, &session->context->errout,
				 "QRTR event_hook: failed to send REG_EVENTS request.\n");
			return;
		}

		ptr = buf;
		hdr = (struct qmi_header *)ptr;
		hdr->type = QMI_REQUEST;
		hdr->txn = txn_id++;
		hdr->msg = QMI_LOC_START;
		hdr->len = sizeof(*tlv) + sizeof(uint8_t);
		ptr += sizeof(*hdr);

		tlv = (struct qmi_tlv *)(buf + sizeof(*hdr));
		tlv->key = QMI_TLV_SESSION_ID;
		tlv->len = sizeof(uint8_t);
		*(uint8_t*)tlv->value = 1;
		ptr += sizeof(*tlv) + sizeof(uint8_t);

		ret = send(sock, buf, ptr - buf, 0);
		if (ret < 0) {
			GPSD_LOG(LOG_ERROR, &session->context->errout,
				 "QRTR event_hook: failed to send START request.\n");
			return;
		}
		break;
	default:
		break;
	}
}

static ssize_t qmi_control_send(struct gps_device_t *session,
                                   char *buf, size_t buflen)
{
    /* do not write if -b (readonly) option set */
    if (session->context->readonly)
        return true;

    session->msgbuflen = buflen;
    (void)memcpy(session->msgbuf, buf, buflen);
    return gpsd_write(session, session->msgbuf, session->msgbuflen);
}

int qmi_pds_open(struct gps_device_t *session)
{
	struct sockaddr_qrtr sq_ctrl;
	socklen_t sl = sizeof(sq_ctrl);
	struct qrtr_ctrl_pkt pkt;
	int flags;
	int sock;
	int ret;
	int i;

	if (session->gpsdata.dev.path == NULL ||
	    strlen(session->gpsdata.dev.path) < QMI_PDS_PATH_STARTS) {
		GPSD_LOG(LOG_ERROR, &session->context->errout,
		"QRTR open: Invalid PDS path.\n");
		return -1;
	}

	for (i = 0; i < QMI_PDS_MAX; i++) {
		if (pds_devices[i] == NULL)
			continue;

		if (strcmp(pds_devices[i]->gpsdata.dev.path,
				session->gpsdata.dev.path) == 0) {
			GPSD_LOG(LOG_ERROR, &session->context->errout,
				"QRTR open: Invalid PDS path already specified.\n");
			return -1;
		}
	}

	for (i = 0; i < QMI_PDS_MAX; i++) {
		if (pds_devices[i] == NULL)
			break;
	}
	if (i == QMI_PDS_MAX) {
		GPSD_LOG(LOG_ERROR, &session->context->errout,
			"QRTR open: Limit of PDS devices reached.\n");
		return -1;
	}
	pds_devices[i] = session;

	sock = socket(AF_QIPCRTR, SOCK_DGRAM, 0);
	if (BAD_SOCKET(sock)) {
	        GPSD_LOG(LOG_ERROR, &session->context->errout,
			 "QRTR open: Unable to get QRTR socket.\n");
		return -1;
	}
	flags = fcntl(sock, F_GETFL, 0);
	flags |= O_NONBLOCK;
	fcntl(sock, F_SETFL, flags);

	ret = getsockname(sock, (struct sockaddr *)&sq_ctrl, &sl);
	if (ret < 0 || sq_ctrl.sq_family != AF_QIPCRTR || sl != sizeof(sq_ctrl)) {
	        GPSD_LOG(LOG_ERROR, &session->context->errout,
			 "QRTR open: Unable to acquire local address.\n");
		close(sock);
		return -1;
	}

	memset(&pkt, 0, sizeof(pkt));
	pkt.cmd = QRTR_TYPE_NEW_LOOKUP;
	pkt.server.service = QMI_PDS_SERVICE_ID;
	pkt.server.instance = QMI_PDS_VERSION;

	sq_ctrl.sq_port = QRTR_PORT_CTRL;
	ret = sendto(sock, &pkt, sizeof(pkt), 0, (struct sockaddr *)&sq_ctrl, sizeof(sq_ctrl));
	if (ret < 0) {
	        GPSD_LOG(LOG_ERROR, &session->context->errout,
			 "QRTR open: Unable to send lookup request.\n");
		close(sock);
		return -1;
	}

	gpsd_switch_driver(session, "Qualcomm PDS");
	session->gpsdata.gps_fd = sock;
	session->sourcetype = source_qrtr;
	session->servicetype = service_sensor;

	return session->gpsdata.gps_fd;
}

void qmi_pds_close(struct gps_device_t *session)
{
	int i;

	if (!BAD_SOCKET(session->gpsdata.gps_fd)) {
		close(session->gpsdata.gps_fd);
		INVALIDATE_SOCKET(session->gpsdata.gps_fd);
	}

	for (i = 0; i < QMI_PDS_MAX; i++) {
		if (pds_devices[i] == NULL)
			continue;

		if (strcmp(pds_devices[i]->gpsdata.dev.path,
				session->gpsdata.dev.path) == 0) {
			pds_devices[i] = NULL;
			break;
		}
	}
}

const struct gps_type_t driver_pds = {
    .type_name      = "Qualcomm PDS",       /* full name of type */
    .packet_type    = NMEA_PACKET,	/* associated lexer packet type */
    .flags	    = DRIVER_STICKY,	/* remember this */
    .channels       = 12,		/* not an actual GPS at all */
    .get_packet     = qmi_pds_get,	/* how to get a packet */
    .parse_packet   = generic_parse_input,	/* how to interpret a packet */
    .event_hook	    = qmi_pds_event_hook,
    .control_send   = qmi_control_send,
};

#endif /* of defined(PDS_ENABLE) */
