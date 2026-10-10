"""Populate provenance without pretending publish time is measurement time."""
from chili_crane_control.evidence_policy import SourceType


def fill_evidence(message, source, measurement_stamp, receive_stamp, counter, age,
                  calibration_id="NOT_CONFIGURED", config_version="NOT_CONFIGURED"):
    metadata = message.evidence
    metadata.header = message.header
    metadata.validity = message.validity
    metadata.reason = message.reason
    metadata.source_type = int(source)
    metadata.measurement_stamp = measurement_stamp
    metadata.receive_stamp = receive_stamp
    metadata.source_counter = counter
    metadata.evidence_age_sec = age
    metadata.calibration_id = calibration_id
    metadata.config_version = config_version
