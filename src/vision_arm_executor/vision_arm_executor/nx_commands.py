"""Pure parsing and deterministic classification for the NX JSON protocol."""

from dataclasses import dataclass

from .station_store import normalize_point_type, normalize_task_command


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
    'vision_station_delete',
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


def _message_point_type(message):
    values = []
    for source in (message, message.get('params', {})):
        if not isinstance(source, dict):
            continue
        for key in ('point_type', 'pointType'):
            if source.get(key) not in (None, ''):
                values.append(normalize_point_type(source.get(key)))
    if values and any(value != values[0] for value in values[1:]):
        raise RuntimeError('conflicting point_type fields')
    return values[0] if values else None


def _message_type(value):
    if not isinstance(value, bool) and value in (1, '1'):
        return 'type1'
    if not isinstance(value, bool) and value in (2, '2'):
        return 'type2'
    if not isinstance(value, bool) and value in (3, '3'):
        return 'type3'
    return str(value or '')


def parse_nx_request(message):
    if not isinstance(message, dict):
        raise RuntimeError('legacy request must be an object')
    params = message.get('params', {})
    if params is not None and not isinstance(params, dict):
        raise RuntimeError('params must be an object')
    message_type = _message_type(message.get('type', ''))

    has_mapid = message.get('mapid') not in (None, '')
    has_poseid = message.get('poseid') not in (None, '')
    if has_mapid != has_poseid:
        raise RuntimeError('mapid and poseid must be provided together')
    has_station = has_mapid and has_poseid

    if message_type in {'execution_enable', 'execution_disable'}:
        return ParsedNxRequest('permission', message, message_type)

    if message_type in EXPLICIT_ACTIONS:
        return ParsedNxRequest(
            'explicit', message, message_type,
            point_type=(
                None if message_type in {
                    'vision_station_execute', 'vision_station_delete'}
                else _message_point_type(message)))

    if message_type == 'type1':
        if not has_station:
            raise RuntimeError('type1 teaching requires mapid and poseid')
        if 'command' in message:
            raise RuntimeError(
                'type1 starts teaching and must not contain command')
        return ParsedNxRequest(
            'teaching_start', message, message_type,
            point_type=_message_point_type(message))

    if message_type == 'type2':
        if not has_station:
            raise RuntimeError('type2 query requires mapid and poseid')
        if 'command' in message:
            raise RuntimeError('type2 must not contain command')
        return ParsedNxRequest('query', message, message_type)

    if message_type == 'type3':
        if not has_station:
            raise RuntimeError('type3 deletion requires mapid and poseid')
        raw_command = message.get('command')
        return ParsedNxRequest(
            'delete', message, message_type,
            command=(None if raw_command in (None, '')
                     else normalize_task_command(raw_command)))

    if message_type == 'mechanical_arm_command':
        if 'command' not in message:
            raise RuntimeError('mechanical_arm_command requires command')
        try:
            command = int(message.get('command'))
        except (TypeError, ValueError):
            raise RuntimeError('command must be an integer')
        if has_station:
            if command < -1:
                raise RuntimeError(
                    'teaching command must be -1, 0 or a positive integer')
            return ParsedNxRequest(
                'teaching_command', message, message_type, command)
        if command not in COMMAND_ACTIONS:
            raise RuntimeError('unsupported legacy command')
        return ParsedNxRequest('command', message, message_type, command)

    # Keep the oldest untyped controller form, but do not let a typed message
    # fall through into an unrelated action.
    if message_type == '' and 'command' in message:
        try:
            command = int(message.get('command'))
        except (TypeError, ValueError):
            raise RuntimeError('command must be an integer')
        if has_station:
            raise RuntimeError(
                'station teaching command requires '
                'type=mechanical_arm_command')
        if command not in COMMAND_ACTIONS:
            raise RuntimeError('unsupported legacy command')
        return ParsedNxRequest('command', message, message_type, command)

    if has_station and message_type == '':
        return ParsedNxRequest('production', message, message_type)

    raise RuntimeError('unsupported NX request')
