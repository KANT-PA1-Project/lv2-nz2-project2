"""Physical marker registry; identity/scale only, never a measured box pose.

Source: user supplied marker-barcode-register (1).png, SHA256
 a3cbdcecf6c2d805d3fe95baf79820dd41fc8bf4ca0249f65da474432be7cfb9.
The register's old puck colour labels are intentionally not imported.
"""
FAMILY = 'tag36h11'
DESTINATION_IDS = {'L1': 12, 'L2': 13, 'L3': 14}


def marker_spec(tag_id):
    """PnP length is the black square, NOT the white sticker perimeter."""
    if type(tag_id) is not int or not 0 <= tag_id <= 14:
        raise ValueError('Unregistered mission marker ID')
    destination = tag_id >= 12
    return {'family': FAMILY, 'id': tag_id,
            'role': 'destination' if destination else 'puck',
            'location': f'L{tag_id - 11}' if destination else None,
            'black_size_m': .048 if destination else .016,
            'including_white_margin_m': .060 if destination else .020}


def destination_spec(location):
    """Top face comes from latest user instruction; geometry is unresolved."""
    if location not in DESTINATION_IDS:
        raise ValueError('Unknown destination')
    return {**marker_spec(DESTINATION_IDS[location]),
            'mount_surface': 'top',
            'mount_source': 'user_instruction_2026-11-05',
            'T_tag_grid': None,
            'mount_geometry_verified': False,
            'requires_sensor_pose': True}
