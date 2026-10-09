"""Manual 0X references are 1-based; PDU offsets are 0-based.

Source: ADAM-6000 appendix B.2.7 and ADAM-6251 register table.
0X uses Read Coils FC01 even when the channel represents an input.
"""


def coil_offset(manual_reference):
    if type(manual_reference) is not int or not 1 <= manual_reference <= 65536:
        raise ValueError("manual coil reference must be 1..65536")
    return manual_reference - 1


class Adam6052RegisterMap:
    DI_COUNT = 8
    DO_COUNT = 8
    DI_MANUAL_START = 1
    DO_MANUAL_START = 17
    DI_OFFSET = coil_offset(DI_MANUAL_START)
    DO_OFFSET = coil_offset(DO_MANUAL_START)


class Adam6251RegisterMap:
    DI_COUNT = 16
    DI_MANUAL_START = 1
    DI_OFFSET = coil_offset(DI_MANUAL_START)
