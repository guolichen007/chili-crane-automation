"""Read-only ADAM-6251 driver (16 DI)."""
from .register_map import Adam6251RegisterMap as Registers


class Adam6251Driver:
    def __init__(self, transport):
        self.transport = transport

    def read_di(self):
        return self.transport.read_coils(Registers.DI_OFFSET, Registers.DI_COUNT)
