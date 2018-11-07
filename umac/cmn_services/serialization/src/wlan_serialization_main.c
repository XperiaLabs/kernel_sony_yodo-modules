/*
 * Copyright (c) 2017-2018 The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * DOC: wlan_serialization_main.c
 * This file defines the important functions pertinent to
 * serialization to initialize and de-initialize the
 * component.
 */
#ifdef CONFIG_SERIALIZATION_V1
#include "qdf_status.h"
#include "qdf_list.h"
#include "wlan_objmgr_cmn.h"
#include "wlan_objmgr_global_obj.h"
#include "wlan_objmgr_psoc_obj.h"
#include "wlan_serialization_main_i.h"
#include "wlan_serialization_rules_i.h"
#include "wlan_serialization_utils_i.h"

struct serialization_legacy_callback ser_legacy_cb;

QDF_STATUS wlan_serialization_psoc_close(struct wlan_objmgr_psoc *psoc)
{
	QDF_STATUS status;
	struct wlan_serialization_psoc_priv_obj *ser_soc_obj =
		wlan_serialization_get_psoc_priv_obj(psoc);

	if (!ser_soc_obj) {
		serialization_err("invalid ser_soc_obj");
		return QDF_STATUS_E_FAILURE;
	}
	/* clean up all timers before exiting */
	status = wlan_serialization_cleanup_all_timers(ser_soc_obj);
	if (status != QDF_STATUS_SUCCESS)
		serialization_err("ser cleanning up all timer failed");

	qdf_mem_free(ser_soc_obj->timers);
	ser_soc_obj->timers = NULL;
	ser_soc_obj->max_active_cmds = 0;

	return status;
}

QDF_STATUS wlan_serialization_psoc_open(struct wlan_objmgr_psoc *psoc)
{
	uint8_t pdev_count;
	struct wlan_serialization_psoc_priv_obj *ser_soc_obj =
		wlan_serialization_get_psoc_priv_obj(psoc);

	if (!ser_soc_obj) {
		serialization_err("invalid ser_soc_obj");
		return QDF_STATUS_E_FAILURE;
	}
	/* TODO:Get WLAN_SERIALIZATION_MAX_ACTIVE_SCAN_CMDS frm service ready */
	pdev_count = wlan_psoc_get_pdev_count(psoc);
	ser_soc_obj->max_active_cmds = WLAN_SERIALIZATION_MAX_ACTIVE_SCAN_CMDS +
		pdev_count;
	ser_soc_obj->timers =
		qdf_mem_malloc(sizeof(struct wlan_serialization_timer) *
				ser_soc_obj->max_active_cmds);
	if (NULL == ser_soc_obj->timers) {
		serialization_alert("Mem alloc failed for ser timers");
		return QDF_STATUS_E_NOMEM;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * wlan_serialization_psoc_obj_create_notification() - PSOC obj create callback
 * @psoc: PSOC object
 * @arg_list: Variable argument list
 *
 * This callback is registered with object manager during initialization and
 * when obj manager gets its turn to create the object, it would notify each
 * component with the corresponding callback registered to inform the
 * completion of the creation of the respective object.
 *
 * Return: QDF Status
 */
static QDF_STATUS wlan_serialization_psoc_obj_create_notification(
		struct wlan_objmgr_psoc *psoc, void *arg_list)
{
	struct wlan_serialization_psoc_priv_obj *soc_ser_obj;

	soc_ser_obj =
		qdf_mem_malloc(sizeof(*soc_ser_obj));
	if (NULL == soc_ser_obj) {
		serialization_alert("Mem alloc failed for ser psoc priv obj");
		return QDF_STATUS_E_NOMEM;
	}
	wlan_objmgr_psoc_component_obj_attach(psoc,
			WLAN_UMAC_COMP_SERIALIZATION, soc_ser_obj,
			QDF_STATUS_SUCCESS);
	serialization_debug("ser psoc obj created");

	return QDF_STATUS_SUCCESS;
}

/**
 * wlan_serialization_destroy_cmd_pool() - Destroy the global cmd pool
 * @ser_pdev_obj: Serialization private pdev object
 *
 * Return: None
 */
static void wlan_serialization_destroy_cmd_pool(
		  struct wlan_serialization_pdev_priv_obj *ser_pdev_obj)
{

	qdf_list_node_t *node = NULL;
	struct wlan_serialization_command_list *cmd_list;

	while (!qdf_list_empty(&ser_pdev_obj->global_cmd_pool_list)) {
		qdf_list_remove_front(&ser_pdev_obj->global_cmd_pool_list,
				&node);
		cmd_list = (struct wlan_serialization_command_list *)node;
		serialization_debug("Node being freed from global pool %pK",
				cmd_list);
		qdf_mem_free(cmd_list);

	}
	qdf_list_destroy(&ser_pdev_obj->global_cmd_pool_list);
}

/**
 * wlan_serialization_create_cmd_pool() - Create the global cmd pool
 * @pdev: PDEV Object
 * @ser_pdev_obj: Serialization private pdev object
 *
 * Global command pool of memory is created here.
 * It is safe to allocate memory individually for each command rather than
 * requesting for a huge chunk of memory at once.
 *
 * The individual command nodes allocated above will keep moving between
 * the active, pending and global pool lists dynamically, but all the
 * memory will be freed during driver unload only.
 *
 * Return: QDF Status
 */
static QDF_STATUS
wlan_serialization_create_cmd_pool(struct wlan_objmgr_pdev *pdev,
		struct wlan_serialization_pdev_priv_obj *ser_pdev_obj)
{
	struct wlan_serialization_command_list *cmd_list_ptr;
	uint8_t i;

	for (i = 0; i < WLAN_SERIALIZATION_MAX_GLOBAL_POOL_CMDS; i++) {
		cmd_list_ptr = qdf_mem_malloc(sizeof(*cmd_list_ptr));
		if (NULL == cmd_list_ptr) {
			serialization_alert("Mem alloc failed for cmd node");
			wlan_serialization_destroy_cmd_pool(ser_pdev_obj);
			return QDF_STATUS_E_NOMEM;
		}
		qdf_list_insert_back(
				&ser_pdev_obj->global_cmd_pool_list,
				&cmd_list_ptr->node);
		cmd_list_ptr->cmd_in_use = 0;
		serialization_debug("Created node at %pK and inserted to pool",
				cmd_list_ptr);
	}

	return QDF_STATUS_SUCCESS;
}


/**
 * wlan_serialization_pdev_obj_create_notification() - PDEV obj create callback
 * @pdev: PDEV object
 * @arg_list: Variable argument list
 *
 * This callback is registered with object manager during initialization and
 * when obj manager gets its turn to create the object, it would notify each
 * component with the corresponding callback registered to inform the
 * completion of the creation of the respective object.
 *
 * Return: QDF Status
 */
static QDF_STATUS wlan_serialization_pdev_obj_create_notification(
		struct wlan_objmgr_pdev *pdev, void *arg_list)
{
	struct wlan_serialization_pdev_priv_obj *ser_pdev_obj;
	QDF_STATUS status;

	ser_pdev_obj =
		qdf_mem_malloc(sizeof(*ser_pdev_obj));
	if (NULL == ser_pdev_obj) {
		serialization_alert("Mem alloc failed for ser pdev obj");
		return QDF_STATUS_E_NOMEM;
	}
	status = wlan_serialization_create_lock(ser_pdev_obj);
	if (status != QDF_STATUS_SUCCESS) {
		serialization_err("Failed to create serialization lock");
		return status;
	}
	qdf_list_create(&ser_pdev_obj->active_list,
			WLAN_SERIALIZATION_MAX_ACTIVE_CMDS);
	qdf_list_create(&ser_pdev_obj->pending_list,
			WLAN_SERIALIZATION_MAX_GLOBAL_POOL_CMDS);
	qdf_list_create(&ser_pdev_obj->active_scan_list,
			WLAN_SERIALIZATION_MAX_ACTIVE_SCAN_CMDS);
	qdf_list_create(&ser_pdev_obj->pending_scan_list,
			WLAN_SERIALIZATION_MAX_GLOBAL_POOL_CMDS);
	qdf_list_create(&ser_pdev_obj->global_cmd_pool_list,
			WLAN_SERIALIZATION_MAX_GLOBAL_POOL_CMDS);
	status = wlan_serialization_create_cmd_pool(pdev, ser_pdev_obj);
	if (status != QDF_STATUS_SUCCESS) {
		serialization_err("ser_pdev_obj failed status %d", status);
		return status;
	}
	status = wlan_objmgr_pdev_component_obj_attach(pdev,
		WLAN_UMAC_COMP_SERIALIZATION, ser_pdev_obj,
		QDF_STATUS_SUCCESS);
	if (status != QDF_STATUS_SUCCESS) {
		serialization_err("serialization pdev obj attach failed");
		return status;
	}

	return status;
}

/**
 * wlan_serialization_psoc_obj_destroy_notification() - PSOC obj delete callback
 * @psoc: PSOC object
 * @arg_list: Variable argument list
 *
 * This callback is registered with object manager during initialization and
 * when obj manager gets its turn to delete the object, it would notify each
 * component with the corresponding callback registered to inform the
 * completion of the deletion of the respective object.
 *
 * Return: QDF Status
 */
static QDF_STATUS wlan_serialization_psoc_obj_destroy_notification(
		struct wlan_objmgr_psoc *psoc, void *arg_list)
{
	QDF_STATUS status;
	struct wlan_serialization_psoc_priv_obj *ser_soc_obj =
		wlan_serialization_get_psoc_priv_obj(psoc);

	if (NULL == ser_soc_obj) {
		serialization_err("invalid ser_soc_obj");
		return QDF_STATUS_E_FAULT;
	}
	status = wlan_objmgr_psoc_component_obj_detach(psoc,
			WLAN_UMAC_COMP_SERIALIZATION,
			ser_soc_obj);
	if (status != QDF_STATUS_SUCCESS)
		serialization_err("ser psoc private obj detach failed");
	serialization_debug("ser psoc obj deleted with status %d", status);
	qdf_mem_free(ser_soc_obj);

	return status;
}

/**
 * wlan_serialization_pdev_obj_destroy_notification() - PDEV obj delete callback
 * @pdev: PDEV object
 * @arg_list: Variable argument list
 *
 * This callback is registered with object manager during initialization and
 * when obj manager gets its turn to delete the object, it would notify each
 * component with the corresponding callback registered to inform the
 * completion of the deletion of the respective object.
 *
 * Return: QDF Status
 */
static QDF_STATUS wlan_serialization_pdev_obj_destroy_notification(
		struct wlan_objmgr_pdev *pdev, void *arg_list)
{
	QDF_STATUS status;
	struct wlan_serialization_pdev_priv_obj *ser_pdev_obj =
		wlan_serialization_get_pdev_priv_obj(pdev);

	status = wlan_objmgr_pdev_component_obj_detach(pdev,
			WLAN_UMAC_COMP_SERIALIZATION, ser_pdev_obj);
	wlan_serialization_destroy_list(ser_pdev_obj,
					&ser_pdev_obj->active_list);
	wlan_serialization_destroy_list(ser_pdev_obj,
					&ser_pdev_obj->pending_list);
	wlan_serialization_destroy_list(ser_pdev_obj,
					&ser_pdev_obj->active_scan_list);
	wlan_serialization_destroy_list(ser_pdev_obj,
					&ser_pdev_obj->pending_scan_list);
	wlan_serialization_destroy_cmd_pool(ser_pdev_obj);
	serialization_debug("ser pdev obj detached with status %d", status);
	status = wlan_serialization_destroy_lock(ser_pdev_obj);
	if (status != QDF_STATUS_SUCCESS)
		serialization_err("Failed to destroy serialization lock");
	qdf_mem_free(ser_pdev_obj);

	return status;
}

/**
 * wlan_serialization_vdev_obj_create_notification() - VDEV obj create callback
 * @vdev: VDEV object
 * @arg_list: Variable argument list
 *
 * This callback is registered with object manager during initialization and
 * when obj manager gets its turn to create the object, it would notify each
 * component with the corresponding callback registered to inform the
 * completion of the creation of the respective object.
 *
 * Return: QDF Status
 */
static QDF_STATUS wlan_serialization_vdev_obj_create_notification(
		struct wlan_objmgr_vdev *vdev, void *arg_list)
{
	return QDF_STATUS_SUCCESS;
}

/**
 * wlan_serialization_vdev_obj_destroy_notification() - vdev obj delete callback
 * @vdev: VDEV object
 * @arg_list: Variable argument list
 *
 * This callback is registered with object manager during initialization and
 * when obj manager gets its turn to delete the object, it would notify each
 * component with the corresponding callback registered to inform the
 * completion of the deletion of the respective object.
 *
 * Return: QDF Status
 */
static QDF_STATUS wlan_serialization_vdev_obj_destroy_notification(
		struct wlan_objmgr_vdev *vdev, void *arg_list)
{
	uint8_t vdev_id = wlan_vdev_get_id(vdev);

	if (!ser_legacy_cb.serialization_purge_cmd_list)
		return QDF_STATUS_SUCCESS;

	serialization_debug("for vdev_id[%d] vdev[%pK] flush all cmds",
			  vdev_id, vdev);
	ser_legacy_cb.serialization_purge_cmd_list(wlan_vdev_get_psoc(vdev),
			vdev, false, false, false, false, true);

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS wlan_serialization_init(void)
{
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	status = wlan_objmgr_register_psoc_create_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_psoc_obj_create_notification, NULL);
	if (status != QDF_STATUS_SUCCESS) {
		serialization_err("Failed to reg soc ser obj create handler");
		goto err_psoc_create;
	}

	status = wlan_objmgr_register_psoc_destroy_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_psoc_obj_destroy_notification, NULL);
	if (status != QDF_STATUS_SUCCESS) {
		serialization_err("Failed to reg soc ser obj delete handler");
		goto err_psoc_delete;
	}

	status = wlan_objmgr_register_pdev_create_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_pdev_obj_create_notification, NULL);
	if (status != QDF_STATUS_SUCCESS) {
		serialization_err("Failed to reg pdev ser obj create handler");
		goto err_pdev_create;
	}

	status = wlan_objmgr_register_pdev_destroy_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_pdev_obj_destroy_notification, NULL);
	if (status != QDF_STATUS_SUCCESS) {
		serialization_err("Failed to reg pdev ser obj delete handler");
		goto err_pdev_delete;
	}

	status = wlan_objmgr_register_vdev_create_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_vdev_obj_create_notification, NULL);
	if (status != QDF_STATUS_SUCCESS) {
		serialization_err("Failed to reg vdev ser obj create handler");
		goto err_vdev_create;
	}

	status = wlan_objmgr_register_vdev_destroy_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_vdev_obj_destroy_notification, NULL);
	if (status != QDF_STATUS_SUCCESS) {
		serialization_err("Failed to reg vdev ser obj delete handler");
		goto err_vdev_delete;
	}
	serialization_debug("serialization handlers registered with obj mgr");
	/*
	 * Initialize the structure so all callbacks are registered
	 * initially as NULL.
	 */
	qdf_mem_zero(&ser_legacy_cb, sizeof(ser_legacy_cb));

	return QDF_STATUS_SUCCESS;

err_vdev_delete:
	wlan_objmgr_unregister_vdev_create_handler(WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_vdev_obj_create_notification, NULL);
err_vdev_create:
	wlan_objmgr_unregister_pdev_destroy_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_pdev_obj_destroy_notification, NULL);
err_pdev_delete:
	wlan_objmgr_unregister_pdev_create_handler(WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_pdev_obj_create_notification, NULL);
err_pdev_create:
	wlan_objmgr_unregister_psoc_destroy_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_psoc_obj_destroy_notification, NULL);
err_psoc_delete:
	wlan_objmgr_unregister_psoc_create_handler(WLAN_UMAC_COMP_SERIALIZATION,
			 wlan_serialization_psoc_obj_create_notification, NULL);
err_psoc_create:
	return status;

}

QDF_STATUS wlan_serialization_deinit(void)
{
	QDF_STATUS status;
	QDF_STATUS ret_status = QDF_STATUS_SUCCESS;

	status = wlan_objmgr_unregister_psoc_create_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_psoc_obj_create_notification,
			NULL);
	if (status != QDF_STATUS_SUCCESS) {
		serialization_err("unreg fail for psoc ser obj create notf:%d",
				status);
		ret_status = QDF_STATUS_E_FAILURE;
	}
	status = wlan_objmgr_unregister_psoc_destroy_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_psoc_obj_destroy_notification,
			NULL);
	if (status != QDF_STATUS_SUCCESS) {
		serialization_err("unreg fail for psoc ser obj destroy notf:%d",
				status);
		ret_status = QDF_STATUS_E_FAILURE;
	}

	status = wlan_objmgr_unregister_pdev_create_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_pdev_obj_create_notification,
			NULL);
	if (status != QDF_STATUS_SUCCESS) {
		serialization_err("unreg fail for pdev ser obj create notf:%d",
				status);
		ret_status = QDF_STATUS_E_FAILURE;
	}

	status = wlan_objmgr_unregister_pdev_destroy_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_pdev_obj_destroy_notification,
			NULL);
	if (status != QDF_STATUS_SUCCESS) {
		serialization_err("unreg fail for pdev ser destroy notf:%d",
				status);
		ret_status = QDF_STATUS_E_FAILURE;
	}

	serialization_alert("deregistered callbacks with obj mgr successfully");
	/*
	 * Initialize the structure so all callbacks are registered
	 * initially as NULL.
	 */
	qdf_mem_zero(&ser_legacy_cb, sizeof(ser_legacy_cb));

	return ret_status;
}
#else /* New serialition code*/
#include <qdf_status.h>
#include <qdf_list.h>
#include <wlan_objmgr_cmn.h>
#include <wlan_objmgr_global_obj.h>
#include <wlan_objmgr_psoc_obj.h>
#include "wlan_serialization_main_i.h"
#include "wlan_serialization_rules_i.h"
#include "wlan_serialization_utils_i.h"

struct serialization_legacy_callback ser_legacy_cb;

QDF_STATUS wlan_serialization_psoc_close(struct wlan_objmgr_psoc *psoc)
{
	QDF_STATUS status = QDF_STATUS_E_FAILURE;
	struct wlan_ser_psoc_obj *ser_soc_obj =
		wlan_serialization_get_psoc_obj(psoc);

	if (!ser_soc_obj) {
		ser_err("invalid ser_soc_obj");
		goto error;
	}
	/* clean up all timers before exiting */
	status = wlan_serialization_cleanup_all_timers(ser_soc_obj);
	if (status != QDF_STATUS_SUCCESS)
		ser_err("ser cleanning up all timer failed");

	qdf_mem_free(ser_soc_obj->timers);
	ser_soc_obj->timers = NULL;
	ser_soc_obj->max_active_cmds = 0;

	wlan_serialization_destroy_lock(&ser_soc_obj->timer_lock);
error:
	return status;
}

QDF_STATUS wlan_serialization_psoc_open(struct wlan_objmgr_psoc *psoc)
{
	uint8_t pdev_count;
	struct wlan_ser_psoc_obj *ser_soc_obj =
		wlan_serialization_get_psoc_obj(psoc);
	QDF_STATUS status = QDF_STATUS_E_FAILURE;

	if (!ser_soc_obj) {
		ser_err("invalid ser_soc_obj");
		goto error;
	}

	pdev_count = wlan_psoc_get_pdev_count(psoc);
	ser_soc_obj->max_active_cmds = WLAN_SER_MAX_ACTIVE_SCAN_CMDS +
					(pdev_count * WLAN_SER_MAX_VDEVS);

	ser_soc_obj->timers =
		qdf_mem_malloc(sizeof(struct wlan_serialization_timer) *
				ser_soc_obj->max_active_cmds);
	if (!ser_soc_obj->timers) {
		ser_alert("Mem alloc failed for ser timers");
		status = QDF_STATUS_E_NOMEM;
		goto error;
	}

	wlan_serialization_create_lock(&ser_soc_obj->timer_lock);
	status = QDF_STATUS_SUCCESS;

error:
	return status;
}

/**
 * wlan_serialization_psoc_create_handler() - PSOC obj create callback
 * @psoc: PSOC object
 * @arg_list: Variable argument list
 *
 * This callback is registered with object manager during initialization and
 * when obj manager gets its turn to create the object, it would notify each
 * component with the corresponding callback registered to inform the
 * completion of the creation of the respective object.
 *
 * Return: QDF Status
 */
static QDF_STATUS wlan_serialization_psoc_create_handler(
		struct wlan_objmgr_psoc *psoc, void *arg_list)
{
	struct wlan_ser_psoc_obj *soc_ser_obj;
	QDF_STATUS status = QDF_STATUS_E_NOMEM;

	soc_ser_obj =
		qdf_mem_malloc(sizeof(*soc_ser_obj));
	if (!soc_ser_obj) {
		ser_alert("Mem alloc failed for ser psoc priv obj");
		goto error;
	}
	status = wlan_objmgr_psoc_component_obj_attach(
					psoc,
					WLAN_UMAC_COMP_SERIALIZATION,
					soc_ser_obj,
					QDF_STATUS_SUCCESS);
	if (QDF_IS_STATUS_ERROR(status)) {
		qdf_mem_free(soc_ser_obj);
		ser_err("Obj attach failed");
		goto error;
	}
	ser_debug("ser psoc obj created");
	status = QDF_STATUS_SUCCESS;

error:
	return status;
}

/**
 * wlan_serialization_destroy_cmd_pool() - Destroy the global cmd pool
 * @ser_pdev_obj: Serialization private pdev object
 *
 * Return: None
 */
static void wlan_serialization_destroy_cmd_pool(
		struct wlan_serialization_pdev_queue *pdev_queue)
{
	qdf_list_node_t *node = NULL;
	struct wlan_serialization_command_list *cmd_list;

	while (!qdf_list_empty(&pdev_queue->cmd_pool_list)) {
		qdf_list_remove_front(&pdev_queue->cmd_pool_list,
				      &node);
		cmd_list = (struct wlan_serialization_command_list *)node;
		ser_debug("Node being freed from global pool %pK",
			  cmd_list);
		qdf_mem_free(cmd_list);
	}

	qdf_list_destroy(&pdev_queue->cmd_pool_list);

}

/**
 * wlan_serialization_create_cmd_pool() - Create the global cmd pool
 * @pdev: PDEV Object
 * @ser_pdev_obj: Serialization private pdev object
 *
 * Global command pool of memory is created here.
 * It is safe to allocate memory individually for each command rather than
 * requesting for a huge chunk of memory at once.
 *
 * The individual command nodes allocated above will keep moving between
 * the active, pending and global pool lists dynamically, but all the
 * memory will be freed during driver unload only.
 *
 * Return: QDF Status
 */
static QDF_STATUS
wlan_serialization_create_cmd_pool(
		struct wlan_serialization_pdev_queue *pdev_queue,
		uint16_t cmd_pool_size)
{
	struct wlan_serialization_command_list *cmd_list_ptr;
	uint8_t i;
	QDF_STATUS status = QDF_STATUS_E_NOMEM;

	qdf_list_create(&pdev_queue->cmd_pool_list, cmd_pool_size);

	for (i = 0; i < cmd_pool_size; i++) {
		cmd_list_ptr = qdf_mem_malloc(sizeof(*cmd_list_ptr));
		if (!cmd_list_ptr) {
			ser_alert("Mem alloc failed for cmd node");
			wlan_serialization_destroy_cmd_pool(pdev_queue);
			goto error;
		}

		qdf_mem_zero(cmd_list_ptr, sizeof(*cmd_list_ptr));
		qdf_list_insert_back(
				     &pdev_queue->cmd_pool_list,
				     &cmd_list_ptr->pdev_node);
		cmd_list_ptr->cmd_in_use = 0;
		ser_debug("Created node at %pK and inserted to pool",
			  cmd_list_ptr);
	}

	status = QDF_STATUS_SUCCESS;

error:
	return status;
}

/**
 * wlan_serialization_pdev_create_handler() - PDEV obj create callback
 * @pdev: PDEV object
 * @arg_list: Variable argument list
 *
 * This callback is registered with object manager during initialization and
 * when obj manager gets its turn to create the object, it would notify each
 * component with the corresponding callback registered to inform the
 * completion of the creation of the respective object.
 *
 * Return: QDF Status
 */
static QDF_STATUS wlan_serialization_pdev_create_handler(
		struct wlan_objmgr_pdev *pdev, void *arg_list)
{
	struct wlan_ser_pdev_obj *ser_pdev_obj;
	struct wlan_serialization_pdev_queue *pdev_queue;
	QDF_STATUS status = QDF_STATUS_E_NOMEM;
	uint8_t index;
	uint8_t free_index;
	uint8_t max_active_cmds;
	uint8_t max_pending_cmds;
	uint16_t cmd_pool_size;

	ser_pdev_obj =
		qdf_mem_malloc(sizeof(*ser_pdev_obj));
	if (!ser_pdev_obj) {
		ser_alert("Mem alloc failed for ser pdev obj");
		goto error;
	}

	for (index = 0; index < SER_PDEV_QUEUE_COMP_MAX; index++) {
		pdev_queue = &ser_pdev_obj->pdev_q[index];

		wlan_serialization_create_lock(&pdev_queue->pdev_queue_lock);

		switch (index) {
		case SER_PDEV_QUEUE_COMP_SCAN:
			max_active_cmds = WLAN_SER_MAX_ACTIVE_SCAN_CMDS;
			max_pending_cmds = WLAN_SER_MAX_PENDING_SCAN_CMDS;
			cmd_pool_size = max_active_cmds + max_pending_cmds;
			break;

		case SER_PDEV_QUEUE_COMP_NON_SCAN:
			max_active_cmds = WLAN_SER_MAX_ACTIVE_CMDS;
			max_pending_cmds = WLAN_SER_MAX_PENDING_CMDS;
			cmd_pool_size = max_active_cmds + max_pending_cmds;
			break;
		}
		qdf_list_create(&pdev_queue->active_list,
				max_active_cmds);
		qdf_list_create(&pdev_queue->pending_list,
				max_pending_cmds);

		status = wlan_serialization_create_cmd_pool(pdev_queue,
							    cmd_pool_size);
		if (status != QDF_STATUS_SUCCESS) {
			ser_err("Create cmd pool failed, status %d", status);
			goto error_free;
		}

		pdev_queue->vdev_active_cmd_bitmap = 0;
		pdev_queue->blocking_cmd_active = 0;
		pdev_queue->blocking_cmd_waiting = 0;
	}

	status = wlan_objmgr_pdev_component_obj_attach(
			pdev, WLAN_UMAC_COMP_SERIALIZATION,
			ser_pdev_obj, QDF_STATUS_SUCCESS);

	if (status != QDF_STATUS_SUCCESS) {
		ser_err("Pdev obj attach failed, status %d", status);
		goto error_free;
	}

	return QDF_STATUS_SUCCESS;

error_free:
	for (free_index = 0; free_index <= index; free_index++) {
		pdev_queue = &ser_pdev_obj->pdev_q[free_index];

		wlan_serialization_destroy_cmd_pool(pdev_queue);
		qdf_list_destroy(&pdev_queue->pending_list);
		qdf_list_destroy(&pdev_queue->active_list);
		wlan_serialization_destroy_lock(&pdev_queue->pdev_queue_lock);
	}
error:
	return status;
}

/**
 * wlan_serialization_psoc_destroy_handler() - PSOC obj delete callback
 * @psoc: PSOC object
 * @arg_list: Variable argument list
 *
 * This callback is registered with object manager during initialization and
 * when obj manager gets its turn to delete the object, it would notify each
 * component with the corresponding callback registered to inform the
 * completion of the deletion of the respective object.
 *
 * Return: QDF Status
 */
static QDF_STATUS
wlan_serialization_psoc_destroy_handler(struct wlan_objmgr_psoc *psoc,
					void *arg_list)
{
	QDF_STATUS status = QDF_STATUS_E_FAULT;
	struct wlan_ser_psoc_obj *ser_soc_obj =
		wlan_serialization_get_psoc_obj(psoc);

	if (!ser_soc_obj) {
		ser_err("invalid ser_soc_obj");
		goto error;
	}
	status = wlan_objmgr_psoc_component_obj_detach(
			psoc, WLAN_UMAC_COMP_SERIALIZATION, ser_soc_obj);
	if (status != QDF_STATUS_SUCCESS)
		ser_err("ser psoc private obj detach failed");

	ser_debug("ser psoc obj deleted with status %d", status);
	qdf_mem_free(ser_soc_obj);

error:
	return status;
}

/**
 * wlan_serialization_pdev_destroy_handler() - PDEV obj delete callback
 * @pdev: PDEV object
 * @arg_list: Variable argument list
 *
 * This callback is registered with object manager during initialization and
 * when obj manager gets its turn to delete the object, it would notify each
 * component with the corresponding callback registered to inform the
 * completion of the deletion of the respective object.
 *
 * Return: QDF Status
 */
static QDF_STATUS wlan_serialization_pdev_destroy_handler(
		struct wlan_objmgr_pdev *pdev, void *arg_list)
{
	QDF_STATUS status;
	struct wlan_serialization_pdev_queue *pdev_queue;
	struct wlan_ser_pdev_obj *ser_pdev_obj =
		wlan_serialization_get_pdev_obj(pdev);
	uint8_t index;

	if (!ser_pdev_obj) {
		ser_err("ser_pdev_obj NULL");
		return QDF_STATUS_E_INVAL;
	}
	status = wlan_objmgr_pdev_component_obj_detach(
			pdev, WLAN_UMAC_COMP_SERIALIZATION, ser_pdev_obj);

	for (index = 0; index < SER_PDEV_QUEUE_COMP_MAX; index++) {
		pdev_queue = &ser_pdev_obj->pdev_q[index];

		wlan_serialization_destroy_pdev_list(pdev_queue);
		wlan_serialization_destroy_cmd_pool(pdev_queue);

		wlan_serialization_destroy_lock(&pdev_queue->pdev_queue_lock);
	}
	qdf_mem_free(ser_pdev_obj);

	return status;
}

/**
 * wlan_serialization_vdev_create_handler() - VDEV obj create callback
 * @vdev: VDEV object
 * @arg_list: Variable argument list
 *
 * This callback is registered with object manager during initialization and
 * when obj manager gets its turn to create the object, it would notify each
 * component with the corresponding callback registered to inform the
 * completion of the creation of the respective object.
 *
 * Return: QDF Status
 */
static QDF_STATUS
wlan_serialization_vdev_create_handler(struct wlan_objmgr_vdev *vdev,
				       void *arg_list)
{
	struct wlan_ser_vdev_obj *ser_vdev_obj;
	struct wlan_serialization_vdev_queue *vdev_q;
	QDF_STATUS status = QDF_STATUS_E_NOMEM;
	uint8_t index;
	uint8_t max_active_cmds;
	uint8_t max_pending_cmds;

	ser_vdev_obj = qdf_mem_malloc(sizeof(*ser_vdev_obj));
	if (!ser_vdev_obj) {
		ser_alert("Mem alloc failed for ser vdev obj");
		goto error;
	}

	for (index = 0; index < SER_VDEV_QUEUE_COMP_MAX; index++) {
		vdev_q = &ser_vdev_obj->vdev_q[index];

		switch (index) {
		case SER_VDEV_QUEUE_COMP_NON_SCAN:
			max_active_cmds = WLAN_SER_MAX_ACTIVE_CMDS /
				WLAN_SER_MAX_VDEVS;
			max_pending_cmds = WLAN_SER_MAX_PENDING_CMDS /
				WLAN_SER_MAX_VDEVS;
			break;
		}

		qdf_list_create(&vdev_q->active_list,
				max_active_cmds);
		qdf_list_create(&vdev_q->pending_list,
				max_pending_cmds);
	}

	status = wlan_objmgr_vdev_component_obj_attach(
			vdev, WLAN_UMAC_COMP_SERIALIZATION, ser_vdev_obj,
			QDF_STATUS_SUCCESS);

	if (status != QDF_STATUS_SUCCESS) {
		for (index = 0; index < SER_VDEV_QUEUE_COMP_MAX; index++) {
			vdev_q = &ser_vdev_obj->vdev_q[index];
			qdf_list_destroy(&vdev_q->pending_list);
			qdf_list_destroy(&vdev_q->active_list);
		}
		qdf_mem_free(ser_vdev_obj);
		ser_err("serialization vdev obj attach failed");
	}
error:
	return status;
}

/**
 * wlan_serialization_vdev_destroy_handler() - vdev obj delete callback
 * @vdev: VDEV object
 * @arg_list: Variable argument list
 *
 * This callback is registered with object manager during initialization and
 * when obj manager gets its turn to delete the object, it would notify each
 * component with the corresponding callback registered to inform the
 * completion of the deletion of the respective object.
 *
 * Return: QDF Status
 */
static QDF_STATUS wlan_serialization_vdev_destroy_handler(
		struct wlan_objmgr_vdev *vdev, void *arg_list)
{
	QDF_STATUS status = QDF_STATUS_SUCCESS;
	struct wlan_serialization_vdev_queue *vdev_q;
	struct wlan_ser_vdev_obj *ser_vdev_obj =
		wlan_serialization_get_vdev_obj(vdev);
	uint8_t vdev_id = wlan_vdev_get_id(vdev);
	uint8_t index;

	if (!ser_vdev_obj) {
		ser_err("ser_vdev_obj NULL");
		return QDF_STATUS_E_INVAL;
	}
	status = wlan_objmgr_vdev_component_obj_detach(
			vdev, WLAN_UMAC_COMP_SERIALIZATION, ser_vdev_obj);

	for (index = 0; index < SER_VDEV_QUEUE_COMP_MAX; index++) {
		vdev_q = &ser_vdev_obj->vdev_q[index];
		wlan_serialization_destroy_vdev_list(&vdev_q->pending_list);
		wlan_serialization_destroy_vdev_list(&vdev_q->active_list);
	}

	qdf_mem_free(ser_vdev_obj);
	if (!ser_legacy_cb.serialization_purge_cmd_list)
		goto error;

	ser_debug("for vdev_id[%d] vdev[%pK] flush all cmds",
		  vdev_id, vdev);
	ser_legacy_cb.serialization_purge_cmd_list(wlan_vdev_get_psoc(vdev),
						   vdev, false, false,
						   false, false, true);

error:
	return status;
}

QDF_STATUS wlan_serialization_init(void)
{
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	status = wlan_objmgr_register_psoc_create_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_psoc_create_handler, NULL);
	if (status != QDF_STATUS_SUCCESS) {
		ser_err("Failed to reg soc ser obj create handler");
		goto err_psoc_create;
	}

	status = wlan_objmgr_register_psoc_destroy_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_psoc_destroy_handler, NULL);
	if (status != QDF_STATUS_SUCCESS) {
		ser_err("Failed to reg soc ser obj delete handler");
		goto err_psoc_delete;
	}

	status = wlan_objmgr_register_pdev_create_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_pdev_create_handler, NULL);
	if (status != QDF_STATUS_SUCCESS) {
		ser_err("Failed to reg pdev ser obj create handler");
		goto err_pdev_create;
	}

	status = wlan_objmgr_register_pdev_destroy_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_pdev_destroy_handler, NULL);
	if (status != QDF_STATUS_SUCCESS) {
		ser_err("Failed to reg pdev ser obj delete handler");
		goto err_pdev_delete;
	}

	status = wlan_objmgr_register_vdev_create_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_vdev_create_handler, NULL);
	if (status != QDF_STATUS_SUCCESS) {
		ser_err("Failed to reg vdev ser obj create handler");
		goto err_vdev_create;
	}

	status = wlan_objmgr_register_vdev_destroy_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_vdev_destroy_handler, NULL);
	if (status != QDF_STATUS_SUCCESS) {
		ser_err("Failed to reg vdev ser obj delete handler");
		goto err_vdev_delete;
	}
	/*
	 * Initialize the structure so all callbacks are registered
	 * initially as NULL.
	 */
	qdf_mem_zero(&ser_legacy_cb, sizeof(ser_legacy_cb));

	status = QDF_STATUS_SUCCESS;
	goto exit;

err_vdev_delete:
	wlan_objmgr_unregister_vdev_create_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_vdev_create_handler,
			NULL);
err_vdev_create:
	wlan_objmgr_unregister_pdev_destroy_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_pdev_destroy_handler,
			NULL);
err_pdev_delete:
	wlan_objmgr_unregister_pdev_create_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_pdev_create_handler,
			NULL);
err_pdev_create:
	wlan_objmgr_unregister_psoc_destroy_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_psoc_destroy_handler,
			NULL);
err_psoc_delete:
	wlan_objmgr_unregister_psoc_create_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_psoc_create_handler,
			NULL);
err_psoc_create:
exit:
	return status;
}

QDF_STATUS wlan_serialization_deinit(void)
{
	QDF_STATUS status;
	QDF_STATUS ret_status = QDF_STATUS_SUCCESS;

	status = wlan_objmgr_unregister_psoc_create_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_psoc_create_handler,
			NULL);

	if (status != QDF_STATUS_SUCCESS) {
		ser_err("unreg fail for psoc ser obj create notf:%d", status);
		ret_status = QDF_STATUS_E_FAILURE;
	}
	status = wlan_objmgr_unregister_psoc_destroy_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_psoc_destroy_handler,
			NULL);

	if (status != QDF_STATUS_SUCCESS) {
		ser_err("unreg fail for psoc ser obj destroy notf:%d", status);
		ret_status = QDF_STATUS_E_FAILURE;
	}

	status = wlan_objmgr_unregister_pdev_create_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_pdev_create_handler,
			NULL);
	if (status != QDF_STATUS_SUCCESS) {
		ser_err("unreg fail for pdev ser obj create notf:%d", status);
		ret_status = QDF_STATUS_E_FAILURE;
	}

	status = wlan_objmgr_unregister_pdev_destroy_handler(
			WLAN_UMAC_COMP_SERIALIZATION,
			wlan_serialization_pdev_destroy_handler,
			NULL);

	if (status != QDF_STATUS_SUCCESS) {
		ser_err("unreg fail for pdev ser destroy notf:%d", status);
		ret_status = QDF_STATUS_E_FAILURE;
	}

	ser_alert("deregistered callbacks with obj mgr successfully");
	/*
	 * Initialize the structure so all callbacks are registered
	 * initially as NULL.
	 */
	qdf_mem_zero(&ser_legacy_cb, sizeof(ser_legacy_cb));

	return ret_status;
}
#endif
