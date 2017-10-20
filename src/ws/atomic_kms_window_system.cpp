/*
 * Copyright © 2017 Collabora Ltd.
 *
 * This file is part of vkmark.
 *
 * vkmark is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation, either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * vkmark is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with vkmark. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:
 *   Alexandros Frantzis <alexandros.frantzis@collabora.com>
 */

#include "atomic_kms_window_system.h"

#include "vulkan_image.h"
#include "vulkan_state.h"

#include "log.h"

#include <xf86drm.h>

#include <system_error>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>

namespace
{

struct ErrnoError : std::system_error
{
    ErrnoError(std::string const& what)
        : std::system_error{errno, std::system_category(), what}
    {
    }
};

bool does_plane_support_crtc_index(drmModePlanePtr plane, uint32_t crtc_index)
{
    return plane->possible_crtcs & (1 << crtc_index);
}

bool is_plane_primary(int drm_fd, drmModePlanePtr plane)
{
    auto const properties = ManagedResource<drmModeObjectPropertiesPtr>{
        drmModeObjectGetProperties(drm_fd, plane->plane_id, DRM_MODE_OBJECT_PLANE),
        [](auto& p) { drmModeFreeObjectProperties(p); }};

    if (!properties)
        throw ErrnoError{"Failed to get plane properties"};

    for (auto i = 0u; i < properties->count_props; ++i)
    {
        auto const property = ManagedResource<drmModePropertyPtr>{
            drmModeGetProperty(drm_fd, properties->props[i]),
            [](auto& p) { drmModeFreeProperty(p); }};

        if (!properties)
            throw ErrnoError{"Failed to get plane property"};

        if (!strcmp(property->name, "type") &&
            properties->prop_values[i] == DRM_PLANE_TYPE_PRIMARY)
        {
            return true;
        }
    }

    return false;
}

ManagedResource<drmModePlanePtr> get_plane_for_crtc(
    int drm_fd, drmModeResPtr resources, drmModeCrtcPtr crtc)
{
    ManagedResource<drmModePlanePtr> ret_plane;

    auto const crtc_index =
        std::distance(
                resources->crtcs,
                std::find(resources->crtcs, resources->crtcs + resources->count_crtcs,
                    crtc->crtc_id));

    auto const plane_resources = ManagedResource<drmModePlaneResPtr>{
        drmModeGetPlaneResources(drm_fd),
        [](auto& p) { drmModeFreePlaneResources(p); }};

    if (!plane_resources)
        throw ErrnoError{"Failed to get plane resources"};

    for (auto i = 0u; i < plane_resources->count_planes; ++i)
    {
        auto plane = ManagedResource<drmModePlanePtr>{
            drmModeGetPlane(drm_fd, plane_resources->planes[i]),
            [] (auto& p) { drmModeFreePlane(p); }};

        if (!plane)
            throw ErrnoError{"Failed to get plane"};

        if (does_plane_support_crtc_index(plane, crtc_index))
        {
            ret_plane = std::move(plane);

            if (is_plane_primary(drm_fd, ret_plane))
                break;
        }
    }

    Log::debug("AtomicKMSWindowSystem: Using plane %d\n",
               ret_plane ? ret_plane->plane_id : -1);

    return ret_plane;
}

bool check_for_atomic_or_throw(int drm_fd)
{
    if (drmSetClientCap(drm_fd, DRM_CLIENT_CAP_ATOMIC, 1) < 0)
        throw std::runtime_error{"Atomic not supported"};

    return true;
}

void page_flip_handler(int, unsigned int, unsigned int, unsigned int, void*)
{
}

}

PropertyIds::PropertyIds(
    int drm_fd,
    drmModeCrtcPtr crtc_ptr,
    drmModeConnectorPtr connector_ptr,
    drmModePlanePtr plane_ptr)
{
    auto const crtc_properties = ManagedResource<drmModeObjectPropertiesPtr>{
        drmModeObjectGetProperties(drm_fd, crtc_ptr->crtc_id, DRM_MODE_OBJECT_CRTC),
        [](auto& p) { drmModeFreeObjectProperties(p); }};
    auto const connector_properties = ManagedResource<drmModeObjectPropertiesPtr>{
        drmModeObjectGetProperties(drm_fd, connector_ptr->connector_id, DRM_MODE_OBJECT_CONNECTOR),
        [](auto& p) { drmModeFreeObjectProperties(p); }};
    auto const plane_properties = ManagedResource<drmModeObjectPropertiesPtr>{
        drmModeObjectGetProperties(drm_fd, plane_ptr->plane_id, DRM_MODE_OBJECT_PLANE),
        [](auto& p) { drmModeFreeObjectProperties(p); }};

    struct { char const* name; int* id_ptr; } const data[] {
        {"FB_ID", &plane.fb_id},
        {"CRTC_ID", &plane.crtc_id},
        {"SRC_X", &plane.src_x},
        {"SRC_Y", &plane.src_y},
        {"SRC_W", &plane.src_w},
        {"SRC_H", &plane.src_h},
        {"CRTC_X", &plane.crtc_x},
        {"CRTC_Y", &plane.crtc_y},
        {"CRTC_W", &plane.crtc_w},
        {"CRTC_H", &plane.crtc_h}
    };

    for (auto const& d : data)
        *d.id_ptr = -1;

    for (auto i = 0u; i < plane_properties->count_props; ++i)
    {
        auto const property = ManagedResource<drmModePropertyPtr>{
            drmModeGetProperty(drm_fd, plane_properties->props[i]),
            [](auto& p) { drmModeFreeProperty(p); }};

        for (auto const& d : data)
        {
            if (!strcmp(property->name, d.name))
                *d.id_ptr = property->prop_id;
        }
    }

    crtc.mode_id = -1;
    crtc.active = -1;

    for (auto i = 0u; i < crtc_properties->count_props; ++i)
    {
        auto const property = ManagedResource<drmModePropertyPtr>{
            drmModeGetProperty(drm_fd, crtc_properties->props[i]),
            [](auto& p) { drmModeFreeProperty(p); }};

        if (!strcmp(property->name, "MODE_ID"))
            crtc.mode_id = property->prop_id;
        else if (!strcmp(property->name, "ACTIVE"))
            crtc.active = property->prop_id;
    }

    connector.crtc_id = -1;

    for (auto i = 0u; i < connector_properties->count_props; ++i)
    {
        auto const property = ManagedResource<drmModePropertyPtr>{
            drmModeGetProperty(drm_fd, connector_properties->props[i]),
            [](auto& p) { drmModeFreeProperty(p); }};

        if (!strcmp(property->name, "CRTC_ID"))
            connector.crtc_id = property->prop_id;
    }
}

bool AtomicKMSWindowSystem::is_supported_on(std::string const& drm_device)
{
    auto const drm_fd = ManagedResource<int>{
        open(drm_device.c_str(), O_RDWR),
        [](auto const fd) { if (fd >= 0) close(fd); }};

    return drm_fd > 0 && drmSetClientCap(drm_fd, DRM_CLIENT_CAP_ATOMIC, 1) == 0;
}

AtomicKMSWindowSystem::AtomicKMSWindowSystem(std::string const& drm_device)
    : KMSWindowSystem(drm_device),
      supports_atomic{check_for_atomic_or_throw(drm_fd)},
      drm_plane{get_plane_for_crtc(drm_fd, drm_resources, drm_crtc)},
      property_ids{drm_fd, drm_crtc, drm_connector, drm_plane}
{
}

void AtomicKMSWindowSystem::present_vulkan_image(VulkanImage const& vulkan_image)
{
    static int constexpr drm_event_context_version = 2;

    static drmEventContext event_context = {
        drm_event_context_version,
        nullptr,
        page_flip_handler};

    auto const& fb_id = drm_fbs[vulkan_image.index];

    // We can't use the VulkanImage semaphore in the KMS window system to
    // synchronize rendering and presentation, so just wait for the graphics
    // queue to finish before flipping.
    vulkan->graphics_queue().waitIdle();

    auto const req = ManagedResource<drmModeAtomicReq*>{
        drmModeAtomicAlloc(),
        [](auto& r) { drmModeAtomicFree(r); }};

    uint32_t flags = DRM_MODE_ATOMIC_NONBLOCK | DRM_MODE_PAGE_FLIP_EVENT;

    if (!has_crtc_been_set)
    {
        uint32_t blob_id;

        drmModeAtomicAddProperty(req, drm_connector->connector_id,
                                 property_ids.connector.crtc_id,
                                 drm_crtc->crtc_id);
        drmModeCreatePropertyBlob(drm_fd, &drm_crtc->mode, sizeof(drm_crtc->mode),
                                  &blob_id);
        drmModeAtomicAddProperty(req, drm_crtc->crtc_id, property_ids.crtc.mode_id, blob_id);
        drmModeAtomicAddProperty(req, drm_crtc->crtc_id, property_ids.crtc.active, 1);

        flags |= DRM_MODE_ATOMIC_ALLOW_MODESET;
        has_crtc_been_set = true;
    }

    auto const plane_id = drm_plane->plane_id;
    drmModeAtomicAddProperty(req, plane_id, property_ids.plane.fb_id, fb_id);
    drmModeAtomicAddProperty(req, plane_id, property_ids.plane.crtc_id, drm_crtc->crtc_id);
    drmModeAtomicAddProperty(req, plane_id, property_ids.plane.src_x, 0);
    drmModeAtomicAddProperty(req, plane_id, property_ids.plane.src_y, 0);
    drmModeAtomicAddProperty(req, plane_id, property_ids.plane.src_w,
                             drm_crtc->mode.hdisplay << 16);
    drmModeAtomicAddProperty(req, plane_id, property_ids.plane.src_h,
                             drm_crtc->mode.vdisplay << 16);
    drmModeAtomicAddProperty(req, plane_id, property_ids.plane.crtc_x, 0);
    drmModeAtomicAddProperty(req, plane_id, property_ids.plane.crtc_y, 0);
    drmModeAtomicAddProperty(req, plane_id, property_ids.plane.crtc_w,
                             drm_crtc->mode.hdisplay);
    drmModeAtomicAddProperty(req, plane_id, property_ids.plane.crtc_h,
                             drm_crtc->mode.vdisplay);


    auto const ret = drmModeAtomicCommit(drm_fd, req, flags, NULL);
    if (ret < 0)
        throw std::system_error{-ret, std::system_category(), "Failed to commit atomic"};

    pollfd pfd{drm_fd, POLLIN, 0};

    while (true)
    {
        auto ret = poll(&pfd, 1, 1000);
        if (ret < 0)
        {
            if (errno == EINTR)
                continue;

            throw std::system_error{
                errno, std::system_category(), "Failed while polling for pages flip event"};
        }

        if (pfd.revents & POLLIN)
        {
            drmHandleEvent(drm_fd, &event_context);
            break;
        }
    }

    current_image_index = (current_image_index + 1) % vk_images.size();
}
