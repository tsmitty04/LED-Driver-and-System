/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "bleprph.h"
#include "services/ans/ble_svc_ans.h"

/*** Maximum number of characteristics with the notify flag ***/
#define MAX_NOTIFY 6
//declares the service UUID to be used for the grow light system with UUID: 59462f12-9543-9999-12c8-58b459a2712d
static const ble_uuid128_t growlightServiceUUID =
    BLE_UUID128_INIT(0x2d, 0x71, 0xa2, 0x59, 0xb4, 0x58, 0xc8, 0x12,
                     0x99, 0x99, 0x43, 0x95, 0x12, 0x2f, 0x46, 0x59);

/* Red Color characteristic that can be subscribed to with UUID: a5ca1f9c-92cb-4f8f-ab7d-3368562cadf4 */
static uint16_t redColorValue;
static uint16_t redColorValueHandle;
static const ble_uuid128_t redColorUUID =
    BLE_UUID128_INIT(0xf4, 0xad, 0x2c, 0x56, 0x68, 0x33, 0x7d, 0xab,
                     0x8f, 0x4f, 0xcb, 0x92, 0x9c, 0x1f, 0xca, 0xa5);

/* Green Color characteristic that can be subscribed to with UUID: 3cc449a6-50a9-40cb-9eea-901d31ea8ae1*/
static uint16_t greenColorValue;
static uint16_t greenColorValueHandle;
static const ble_uuid128_t greenColorUUID =
    BLE_UUID128_INIT(0xe1, 0x8a, 0xea, 0x31, 0x1d, 0x90, 0xea, 0x9e,
                     0xcb, 0x40, 0xa9, 0x50, 0xa6, 0x49, 0xc4, 0x3c);

/* Blue Color characteristic that can be subscribed to with UUID: e669b089-cc6e-4fda-8893-316925591df6*/
static uint16_t blueColorValue;
static uint16_t blueColorValueHandle;
static const ble_uuid128_t blueColorUUID =
    BLE_UUID128_INIT(0xf6, 0x1d, 0x59, 0x25, 0x69, 0x31, 0x93, 0x88,
                     0xda, 0x4f, 0x6e, 0xcc, 0x89, 0xb0, 0x69, 0xe6);

/* on/Off characteristic that can be subscribed to with UUID: 51be37e8-53c8-40dd-89f5-787170826c07*/
static uint8_t onOffValue;
static uint16_t onOffValueHandle;
static const ble_uuid128_t onOffValueUUID =
    BLE_UUID128_INIT(0x07, 0x6c, 0x82, 0x70, 0x71, 0x78, 0xf5, 0x89,
                     0xdd, 0x40, 0xc8, 0x53, 0xe8, 0x37, 0xbe, 0x51);

/* timerInfo characteristic that can be subscribed to with UUID: 2bcc2532-4628-4746-920e-867d76a10845*/
static uint16_t timerInfoValue;
static uint16_t timerInfoValueHandle;
static const ble_uuid128_t timerInfoUUID =
    BLE_UUID128_INIT(0x45, 0x08, 0xa1, 0x76, 0x7d, 0x86, 0x0e, 0x92,
                     0x46, 0x47, 0x28, 0x46, 0x32, 0x25, 0xcc, 0x2b);

/* lumensDisplay characteristic that can be subscribed to with UUID: 5432ae84-6a51-4b93-84f0-e69056d654ef */
static uint16_t lumensDisplayValue;
static uint16_t lumensDisplayValueHandle;
static const ble_uuid128_t lumensDisplayUUID =
    BLE_UUID128_INIT(0xef, 0x54, 0xd6, 0x56, 0x90, 0xe6, 0xf0, 0x84,
                     0x93, 0x4b, 0x51, 0x6a, 0x84, 0xae, 0x32, 0x54);

               
// A custom descriptor
static uint8_t gatt_svr_dsc_val;
static const ble_uuid128_t gatt_svr_dsc_uuid =
    BLE_UUID128_INIT(0x01, 0x01, 0x01, 0x01, 0x12, 0x12, 0x12, 0x12,
                     0x23, 0x23, 0x23, 0x23, 0x34, 0x34, 0x34, 0x34);

static int
gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                struct ble_gatt_access_ctxt *ctxt,
                void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = 
{
    {
        /*** Service ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &growlightServiceUUID.u,
        .characteristics = (struct ble_gatt_chr_def[])
        { 
            {
                /*** these characteristics can be subscribed to by writing 0x00 and 0x01 to the CCCD ***/
                .uuid = &redColorUUID.u,
                .access_cb = gatt_svc_access,
#if CONFIG_EXAMPLE_ENCRYPTION
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
                BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#else
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#endif
                .val_handle = &redColorValueHandle,
                .descriptors = (struct ble_gatt_dsc_def[])
                { 
                    {
                      .uuid = &gatt_svr_dsc_uuid.u,
#if CONFIG_EXAMPLE_ENCRYPTION
                      .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
#else
                      .att_flags = BLE_ATT_F_READ,
#endif
                      .access_cb = gatt_svc_access,
                    }, 
                    {
                      0, /* No more descriptors in this characteristic */
                    }
                },
            }, 
            
            {
                .uuid = &greenColorUUID.u,
                .access_cb = gatt_svc_access,
#if CONFIG_EXAMPLE_ENCRYPTION
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
                BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#else
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#endif
                .val_handle = &greenColorValueHandle,
                .descriptors = (struct ble_gatt_dsc_def[])
                { 
                    {
                      .uuid = &gatt_svr_dsc_uuid.u,
#if CONFIG_EXAMPLE_ENCRYPTION
                      .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
#else
                      .att_flags = BLE_ATT_F_READ,
#endif
                      .access_cb = gatt_svc_access,
                    }, 
                    {
                      0, /* No more descriptors in this characteristic */
                    }
                },
            },

            {
                .uuid = &blueColorUUID.u,
                .access_cb = gatt_svc_access,
#if CONFIG_EXAMPLE_ENCRYPTION
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
                BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#else
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#endif
                .val_handle = &blueColorValueHandle,
                .descriptors = (struct ble_gatt_dsc_def[])
                { 
                    {
                      .uuid = &gatt_svr_dsc_uuid.u,
#if CONFIG_EXAMPLE_ENCRYPTION
                      .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
#else
                      .att_flags = BLE_ATT_F_READ,
#endif
                      .access_cb = gatt_svc_access,
                    }, 
                    {
                      0, /* No more descriptors in this characteristic */
                    }
                },
            },

            {
                .uuid = &onOffValueUUID.u,
                .access_cb = gatt_svc_access,
#if CONFIG_EXAMPLE_ENCRYPTION
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
                BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#else
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#endif
                .val_handle = &onOffValueHandle,
                .descriptors = (struct ble_gatt_dsc_def[])
                { 
                    {
                      .uuid = &gatt_svr_dsc_uuid.u,
#if CONFIG_EXAMPLE_ENCRYPTION
                      .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
#else
                      .att_flags = BLE_ATT_F_READ,
#endif
                      .access_cb = gatt_svc_access,
                    }, 
                    {
                      0, /* No more descriptors in this characteristic */
                    }

                },
            },

            {
                .uuid = &timerInfoUUID.u,
                .access_cb = gatt_svc_access,
#if CONFIG_EXAMPLE_ENCRYPTION
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
                BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#else
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#endif
                .val_handle = &timerInfoValueHandle,
                .descriptors = (struct ble_gatt_dsc_def[])
                { 
                    {
                      .uuid = &gatt_svr_dsc_uuid.u,
#if CONFIG_EXAMPLE_ENCRYPTION
                      .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
#else
                      .att_flags = BLE_ATT_F_READ,
#endif
                      .access_cb = gatt_svc_access,
                    }, 
                    {
                      0, /* No more descriptors in this characteristic */
                    }
                },
            },

            {
                .uuid = &lumensDisplayUUID.u,
                .access_cb = gatt_svc_access,
#if CONFIG_EXAMPLE_ENCRYPTION
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
                BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#else
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#endif
                .val_handle = &lumensDisplayValueHandle,
                .descriptors = (struct ble_gatt_dsc_def[])
                { 
                    {
                      .uuid = &gatt_svr_dsc_uuid.u,
#if CONFIG_EXAMPLE_ENCRYPTION
                      .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
#else
                      .att_flags = BLE_ATT_F_READ,
#endif
                      .access_cb = gatt_svc_access,
                    }, 
                    {
                      0, /* No more descriptors in this characteristic */
                    }
                },    
            },

            {
                0, //no more characteristics in this service
            },
        }
    
    },

    {
            0, /* No more services. */
    },
};

static int
gatt_svr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
               void *dst, uint16_t *len)
{
    uint16_t om_len;
    int rc;

    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}

/**
 * Access callback whenever a characteristic/descriptor is read or written to.
 * Here reads and writes need to be handled.
 * ctxt->op tells weather the operation is read or write and
 * weather it is on a characteristic or descriptor,
 * ctxt->dsc->uuid tells which characteristic/descriptor is accessed.
 * attr_handle give the value handle of the attribute being accessed.
 * Accordingly do:
 *     Append the value to ctxt->om if the operation is READ
 *     Write ctxt->om to the value if the operation is WRITE
 **/
static int
gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const ble_uuid_t *uuid;
    int rc;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            MODLOG_DFLT(INFO, "Characteristic read; conn_handle=%d attr_handle=%d\n",
                        conn_handle, attr_handle);
        } else {
            MODLOG_DFLT(INFO, "Characteristic read by NimBLE stack; attr_handle=%d\n",
                        attr_handle);
        }
        uuid = ctxt->chr->uuid;
        if (attr_handle == redColorValueHandle) {
            rc = os_mbuf_append(ctxt->om,
                                &redColorValue,
                                sizeof(redColorValue));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        else if (attr_handle == greenColorValueHandle) {
            rc = os_mbuf_append(ctxt->om,
                                &greenColorValue,
                                sizeof(greenColorValue));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        else if (attr_handle == blueColorValueHandle) {
            rc = os_mbuf_append(ctxt->om,
                                &blueColorValue,
                                sizeof(blueColorValue));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        else if (attr_handle == onOffValueHandle) {
            rc = os_mbuf_append(ctxt->om,
                                &onOffValue,
                                sizeof(onOffValue));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        else if (attr_handle == timerInfoValueHandle) {
            rc = os_mbuf_append(ctxt->om,
                                &timerInfoValue,
                                sizeof(timerInfoValue));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        else if (attr_handle == lumensDisplayValueHandle) {
            rc = os_mbuf_append(ctxt->om,
                                &lumensDisplayValue,
                                sizeof(lumensDisplayValue));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        else {
            goto unknown;
        }
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            MODLOG_DFLT(INFO, "Characteristic write; conn_handle=%d attr_handle=%d",
                        conn_handle, attr_handle);
        } else {
            MODLOG_DFLT(INFO, "Characteristic write by NimBLE stack; attr_handle=%d",
                        attr_handle);
        }
        uuid = ctxt->chr->uuid;
        if (attr_handle == redColorValueHandle) {
            rc = gatt_svr_write(ctxt->om,
                                sizeof(redColorValue),
                                sizeof(redColorValue),
                                &redColorValue, NULL);
            ble_gatts_chr_updated(attr_handle);
            MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                        "all subscribed peers.\n");
            return rc;
        }
        else if (attr_handle == greenColorValueHandle) {
            rc = gatt_svr_write(ctxt->om,
                                sizeof(greenColorValue),
                                sizeof(greenColorValue),
                                &greenColorValue, NULL);
            ble_gatts_chr_updated(attr_handle);
            MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                        "all subscribed peers.\n");
            return rc;
        }
        else if (attr_handle == blueColorValueHandle) {
            rc = gatt_svr_write(ctxt->om,
                                sizeof(blueColorValue),
                                sizeof(blueColorValue),
                                &blueColorValue, NULL);
            ble_gatts_chr_updated(attr_handle);
            MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                        "all subscribed peers.\n");
            return rc;
        }    
        else if (attr_handle == onOffValueHandle) {
            rc = gatt_svr_write(ctxt->om,
                                sizeof(onOffValue),
                                sizeof(onOffValue),
                                &onOffValue, NULL);
            ble_gatts_chr_updated(attr_handle);
            MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                        "all subscribed peers.\n");
            return rc;
        }
        else if (attr_handle == timerInfoValueHandle) {
            rc = gatt_svr_write(ctxt->om,
                                sizeof(timerInfoValue),
                                sizeof(timerInfoValue),
                                &timerInfoValue, NULL);
            ble_gatts_chr_updated(attr_handle);
            MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                        "all subscribed peers.\n");
            return rc;
        }
        else if (attr_handle == lumensDisplayValueHandle) {
            rc = gatt_svr_write(ctxt->om,
                                sizeof(lumensDisplayValue),
                                sizeof(lumensDisplayValue),
                                &lumensDisplayValue, NULL);
            ble_gatts_chr_updated(attr_handle);
            MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                        "all subscribed peers.\n");
            return rc;
        }
        else {
            goto unknown;
        }

    case BLE_GATT_ACCESS_OP_READ_DSC:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            MODLOG_DFLT(INFO, "Descriptor read; conn_handle=%d attr_handle=%d\n",
                        conn_handle, attr_handle);
        } else {
            MODLOG_DFLT(INFO, "Descriptor read by NimBLE stack; attr_handle=%d\n",
                        attr_handle);
        }
        uuid = ctxt->dsc->uuid;
        if (ble_uuid_cmp(uuid, &gatt_svr_dsc_uuid.u) == 0) {
            rc = os_mbuf_append(ctxt->om,
                                &gatt_svr_dsc_val,
                                sizeof(gatt_svr_dsc_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        else {
            goto unknown;
        }
    case BLE_GATT_ACCESS_OP_WRITE_DSC:
        goto unknown;

    default:
        goto unknown;
    }

unknown:
    /* Unknown characteristic/descriptor;
     * The NimBLE host should not have called this function;
     */
    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
}

void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

int
gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_ans_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    /* Setting a value for the read-only descriptor */
    gatt_svr_dsc_val = 0x1234;

    return 0;
}