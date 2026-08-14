"""Pure parsing and classification for the legacy NX JSON protocol."""

from dataclasses import dataclass

from .station_store import normalize_point_type


COMMAND_ACTIONS = {
    1: 'robot_reset',
    2: 'robot_enable',
    3: 'robot_disable',
    4: 'robot_start_drag',
    5: 'robot_stop_drag',
    6: 'robot_clear_error',
}

EXPLICIT_ACTIONS = {
    'vision_point_teach',
    'vision_station_points',
    'vision_station_execute',
    'vision_icp_align',
    'vision_icp_align_and_move_b',
    'apriltag_locate',
    'apriltag_locate_and_pick',
    'apriltag_locate_and_touch',
    'apriltag_validate',
    'apriltag_pick',
    'arm_status',
    'execution_enable',
    'execution_disable',
    'robot_start_drag',
    'robot_stop_drag',
}


@dataclass(frozen=True)
class ParsedNxRequest:
    kind: str
    message: dict
    message_type: str
    command: object = None
    point_type: object = None


def _message_point_type(message, required=False):
    values = []
    for source in (message, message.get('params', {})):
        if not isinstance(source, dict):
            continue
        for key in ('point_type', 'pointType'):
            if source.get(key) not in (None, ''):
                values.append(normalize_point_type(source.get(key)))
    if values and any(value != values[0] for value in values[1:]):
        raise RuntimeError('conflicting point_type fields')
    if values:
        return values[0]
    return normalize_point_type(None, required=required)


def parse_nx_request(message):
    if not isinstance(message, dict):
        raise RuntimeError('legacy request must be an object')
    params = message.get('params', {})
    if params is not None and not isinstance(params, dict):
        raise RuntimeError('params must be an object')
    raw_type = message.get('type', '')
    # The historical clients use both "type1"/"type2" and numeric 1/2.
    # Normalize them before classification so {"type": 2, ...} cannot fall
    # through to the mapid+poseid production route.
    if not isinstance(raw_type, bool) and raw_type in (1, '1'):
        message_type = 'type1'
    elif not isinstance(raw_type, bool) and raw_type in (2, '2'):
        message_type = 'type2'
    else:
        message_type = str(raw_type or '')

    if message_type in {'execution_enable', 'execution_disable'}:
        return ParsedNxRequest('permission', message, message_type)

    # Preserve the old controller's precedence: an untyped JSON containing
    # command=1..6 is a control command. Explicit executor actions keep their
    # own command field semantics.
    if 'command' in message and message_type not in EXPLICIT_ACTIONS:
        try:
            command = int(message.get('command'))
        except (TypeError, ValueError):
            raise RuntimeError('command must be an integer from 1 to 6')
        if command not in COMMAND_ACTIONS:
            raise RuntimeError('unsupported legacy command')
        return ParsedNxRequest('command', message, message_type, command)

    if message_type in EXPLICIT_ACTIONS:
        return ParsedNxRequest(
            'explicit', message, message_type,
            point_type=(
                None if message_type == 'vision_station_execute'
                else _message_point_type(message, required=False)))

    if message_type == 'type2':
        return ParsedNxRequest('query', message, message_type)

    if message_type in {'demo_point_recorded', 'type1'}:
        return ParsedNxRequest(
            'teaching_start', message, message_type,
            point_type=_message_point_type(message, required=True))

    if message.get('mapid') not in (None, '') and message.get(
            'poseid') not in (None, ''):
        return ParsedNxRequest('production', message, message_type)

    raise RuntimeError('unsupported NX request')
