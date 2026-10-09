"""Pure logical DO mapping; not a writer and not a physical permission gate."""


def request_to_do(request):
    outputs = [False] * 8
    if request.enable is not True:
        return tuple(outputs)
    if request.speed_command_valid is True:
        raise ValueError("ADAM direction-only interface cannot execute speed setpoints")
    pairs = {"Y": (0, 1), "Z": (3, 2), "G": (5, 4)}
    if request.axis not in pairs or type(request.direction) is not int or request.direction not in (-1, 1):
        raise ValueError("axis/direction not mapped to ADAM")
    outputs[pairs[request.axis][int(request.direction > 0)]] = True
    return tuple(outputs)
