"""Adapter to externally installed official MVS Python bindings; no copied vendor ABI."""
import ctypes
import importlib
import os
from pathlib import Path
import socket
import sys

FEATURES = {"GevIEEE1588": "bool", "GevIEEE1588Status": "enum", "GevIEEE1588SlaveOnly": "bool",
            "GevTimestampTickFrequency": "int", "GevTimestampValue": "int"}


def load_sdk():
    path = os.environ.get("MVS_PYTHON_PATH", "")
    if not path or not Path(path).is_dir():
        raise RuntimeError("MVS_SDK_NOT_CONFIGURED: set MVS_PYTHON_PATH to official MvImport directory")
    sys.path.insert(0, path)
    return importlib.import_module("MvCameraControl_class")


def cstring(data):
    return bytes(data).split(b"\0", 1)[0].decode("utf-8", errors="replace")


class MvsCamera:
    def __init__(self, cfg):
        self.sdk, self.cfg = load_sdk(), cfg
        self.cam = self.sdk.MvCamera()
        self.opened = self.grabbing = False
        expected_serial, expected_ip = cfg.get("expected_serial"), cfg.get("expected_ip")
        if expected_serial in (None, "", "NOT_CONFIGURED") and expected_ip in (None, "", "NOT_CONFIGURED"):
            raise ValueError("explicit camera identity required; never first-seen-wins")
        devices = self.sdk.MV_CC_DEVICE_INFO_LIST()
        self.check(self.sdk.MvCamera.MV_CC_EnumDevices(self.sdk.MV_GIGE_DEVICE, devices), "enumerate")
        candidates = []
        for i in range(devices.nDeviceNum):
            dev = ctypes.cast(devices.pDeviceInfo[i], ctypes.POINTER(self.sdk.MV_CC_DEVICE_INFO)).contents
            if dev.nTLayerType != self.sdk.MV_GIGE_DEVICE:
                continue
            data = dev.SpecialInfo.stGigEInfo
            serial, model = cstring(data.chSerialNumber), cstring(data.chModelName)
            ip = socket.inet_ntoa(int(data.nCurrentIp).to_bytes(4, "big"))
            if ((expected_serial in (None, "", "NOT_CONFIGURED") or serial == expected_serial)
                    and (expected_ip in (None, "", "NOT_CONFIGURED") or ip == expected_ip)
                    and model == cfg.get("model", "MV-CS060-10GC")):
                candidates.append((dev, serial, model, ip))
        if len(candidates) != 1:
            raise RuntimeError("CAMERA_IDENTITY_NOT_UNIQUE_OR_NOT_FOUND")
        self.device, self.serial, self.model, self.ip = candidates[0]
        self.check(self.cam.MV_CC_CreateHandle(self.device), "create handle")
        try:
            self.check(self.cam.MV_CC_OpenDevice(self.sdk.MV_ACCESS_Exclusive, 0), "open")
            self.opened = True
        except Exception:
            self.cam.MV_CC_DestroyHandle()
            raise

    @staticmethod
    def check(ret, operation):
        if ret != 0:
            raise RuntimeError(operation + ": SDK code " + hex(ret & 0xFFFFFFFF))

    def feature(self, name, kind):
        value = None
        try:
            if kind == "bool":
                out = ctypes.c_bool()
                ret = self.cam.MV_CC_GetBoolValue(name, out)
                value = bool(out.value)
            elif kind == "enum":
                out = self.sdk.MVCC_ENUMVALUE()
                ret = self.cam.MV_CC_GetEnumValue(name, out)
                value = int(out.nCurValue)
                entry_type = getattr(self.sdk, "MVCC_ENUMENTRY", None)
                entry_fn = getattr(self.cam, "MV_CC_GetEnumEntrySymbolic", None)
                if ret == 0 and entry_type and entry_fn:
                    entry = entry_type()
                    entry.nValue = out.nCurValue
                    if entry_fn(name, entry) == 0:
                        value = cstring(entry.chSymbolic)
            elif kind == "float":
                out = self.sdk.MVCC_FLOATVALUE()
                ret = self.cam.MV_CC_GetFloatValue(name, out)
                value = float(out.fCurValue)
            else:
                out = self.sdk.MVCC_INTVALUE_EX()
                ret = self.cam.MV_CC_GetIntValueEx(name, out)
                value = int(out.nCurValue)
        except (AttributeError, TypeError):
            return {"support": "SDK_API_UNAVAILABLE", "access": "UNKNOWN", "current_value": None}
        # A getter failure is not sufficient proof that a node does not exist.
        access, support = "UNKNOWN", "SUPPORTED" if ret == 0 else "PROBE_FAILED"
        fn = getattr(self.cam, "MV_XML_GetNodeAccessMode", None)
        if fn:
            mode = ctypes.c_uint()
            try:
                access_ret = fn(name, mode)
                if access_ret == 0:
                    for symbol, label in (("AM_RO", "READ_ONLY"), ("AM_RW", "WRITEABLE"),
                                          ("AM_WO", "WRITEABLE"), ("AM_NI", "NOT_SUPPORTED"), ("AM_NA", "NOT_AVAILABLE")):
                        if hasattr(self.sdk, symbol) and mode.value == getattr(self.sdk, symbol):
                            access = label
                            if label == "NOT_SUPPORTED":
                                support = label
            except (AttributeError, TypeError):
                pass
        return {"support": support, "access": access, "current_value": value if ret == 0 else None,
                "sdk_return_code": ret}

    def probe(self):
        return {name: self.feature(name, kind) for name, kind in FEATURES.items()}

    def configure(self):
        cfg = self.cfg
        if not cfg.get("configure_camera"):
            fps = self.feature("AcquisitionFrameRate", "float").get("current_value")
            if fps is None or fps > cfg["maximum_device_fps"]:
                raise RuntimeError("LOW_BANDWIDTH_CONFIGURATION_REQUIRES_OWNER_CONFIRMATION")
            return
        rate = cfg.get("requested_frame_rate_hz")
        if not isinstance(rate, (float, int)) or not 0 < rate <= cfg["maximum_device_fps"]:
            raise ValueError("bounded explicit camera frame rate required")
        self.check(self.cam.MV_CC_SetBoolValue("AcquisitionFrameRateEnable", True), "frame rate enable")
        self.check(self.cam.MV_CC_SetFloatValue("AcquisitionFrameRate", rate), "frame rate")
        fmt = cfg.get("requested_pixel_format")
        if fmt not in {"Mono8", "BayerRG8", "BayerBG8", "BayerGR8", "BayerGB8"}:
            raise ValueError("explicit raw8 pixel format required")
        self.check(self.cam.MV_CC_SetEnumValueByString("PixelFormat", fmt), "pixel format")
        for name, value in (cfg.get("roi") or {}).items():
            if name not in {"OffsetX", "OffsetY", "Width", "Height"} or type(value) is not int or value < 0:
                raise ValueError("invalid ROI")
            self.check(self.cam.MV_CC_SetIntValueEx(name, value), name)
        for name, value in (cfg.get("binning") or {}).items():
            if name not in {"BinningHorizontal", "BinningVertical"} or type(value) is not int or not 1 <= value <= 8:
                raise ValueError("invalid binning")
            self.check(self.cam.MV_CC_SetIntValueEx(name, value), name)
        if cfg.get("freeze_exposure"):
            exposure = cfg.get("exposure_time")
            if not isinstance(exposure, (float, int)) or exposure <= 0:
                raise ValueError("explicit exposure required")
            self.check(self.cam.MV_CC_SetEnumValueByString("ExposureAuto", "Off"), "auto exposure")
            self.check(self.cam.MV_CC_SetFloatValue("ExposureTime", exposure), "exposure")

    def start(self):
        self.configure()
        self.check(self.cam.MV_CC_StartGrabbing(), "start grabbing")
        self.grabbing = True

    def frame(self, timeout_ms=100):
        out = self.sdk.MV_FRAME_OUT()
        ret = self.cam.MV_CC_GetImageBuffer(out, timeout_ms)
        if ret != 0:
            return None
        try:
            info = out.stFrameInfo
            keys = ("nDevTimeStampHigh", "nDevTimeStampLow", "nHostTimeStamp", "nFrameNum", "fExposureTime", "nLostPacket")
            data = {k: getattr(info, k) for k in keys}
            data.update(width=int(info.nWidth), height=int(info.nHeight), pixel_format=int(info.enPixelType))
            # Only uncompressed 8-bit raw; no silent BGR conversion or guessed layout.
            encoding = None
            for suffix, name in (("Mono8", "mono8"), ("BayerRG8", "bayer_rggb8"), ("BayerBG8", "bayer_bggr8"),
                                 ("BayerGR8", "bayer_grbg8"), ("BayerGB8", "bayer_gbrg8")):
                if getattr(self.sdk, "PixelType_Gvsp_" + suffix, None) == info.enPixelType:
                    encoding = name
            length = int(info.nFrameLen)
            if encoding is None or length != data["width"] * data["height"] or not 0 < length <= 16 * 1024 * 1024:
                raise RuntimeError("UNSUPPORTED_RAW_PIXEL_LAYOUT")
            return data, ctypes.string_at(out.pBufAddr, length), encoding
        finally:
            self.cam.MV_CC_FreeImageBuffer(out)

    def close(self):
        if self.grabbing:
            self.cam.MV_CC_StopGrabbing()
        if self.opened:
            self.cam.MV_CC_CloseDevice()
        self.cam.MV_CC_DestroyHandle()
