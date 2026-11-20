#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "smartcar/hybrid.hpp"
#include <filesystem>
#include <stdexcept>

namespace smartcar {
namespace {
struct PyRelease {
    void operator()(PyObject *p) const { Py_XDECREF(p); }
};
using Owned = std::unique_ptr<PyObject, PyRelease>;
Owned checked(PyObject *raw) {
    if (!raw) {
        PyErr_Print();
        throw std::runtime_error("legacy vision failed; see Python traceback");
    }
    return Owned(raw);
}
Owned pixels(const cv::Mat &frame) {
    if (frame.empty()) {
        Py_INCREF(Py_None);
        return Owned(Py_None);
    }
    if (frame.size() != cv::Size(480, 360) || frame.type() != CV_8UC3)
        throw std::runtime_error("legacy vision expects an uncropped 480x360 BGR view");
    cv::Mat data = frame.isContinuous() ? frame : frame.clone();
    return checked(PyBytes_FromStringAndSize(reinterpret_cast<const char *>(data.data),
                                             static_cast<Py_ssize_t>(data.total() * data.elemSize())));
}
void startPython() {
    if (Py_IsInitialized())
        return;
    PyConfig cfg;
    PyConfig_InitPythonConfig(&cfg);
    cfg.install_signal_handlers = 0;
    cfg.write_bytecode = 0;
    auto exe = std::filesystem::u8path(HYBRID_PYTHON_EXECUTABLE).wstring();
    auto status = PyConfig_SetString(&cfg, &cfg.program_name, exe.c_str());
    if (!PyStatus_Exception(status))
        status = PyConfig_SetString(&cfg, &cfg.executable, exe.c_str());
    if (!PyStatus_Exception(status))
        status = Py_InitializeFromConfig(&cfg);
    std::string error = PyStatus_Exception(status) ? (status.err_msg ? status.err_msg : "init failed") : "";
    PyConfig_Clear(&cfg);
    if (!error.empty())
        throw std::runtime_error("Python initialization: " + error);
}
} // namespace
struct HybridVision::Impl {
    Owned object;
};
HybridVision::HybridVision(const std::string &modules, const std::string &config, const std::string &primary,
                           const std::string &secondary)
    : impl_(std::make_unique<Impl>()) {
    startPython();
    auto site = checked(PyImport_ImportModule("site"));
    auto loaded = checked(PyObject_CallMethod(site.get(), "addsitedir", "s", HYBRID_PYTHON_PACKAGES));
    auto root = checked(PyUnicode_FromString(modules.c_str()));
    if (PyList_Insert(PySys_GetObject("path"), 0, root.get()) < 0)
        throw std::runtime_error("cannot set algorithm path");
    auto module = checked(PyImport_ImportModule("hybrid"));
    auto cls = checked(PyObject_GetAttrString(module.get(), "Hybrid"));
    impl_->object =
        checked(PyObject_CallFunction(cls.get(), "sss", config.c_str(), primary.c_str(), secondary.c_str()));
}
HybridVision::~HybridVision() = default;
HybridEvidence HybridVision::update(const std::string &stage, const cv::Mat &road, const cv::Mat &primary,
                                    std::uint64_t pid, double pt, double now, const Config &c,
                                    Observation &o) {
    auto rb = pixels(road), pb = pixels(primary);
    auto result = checked(PyObject_CallMethod(impl_->object.get(), "step", "sOKdOKdd", stage.c_str(),
                                              rb.get(), static_cast<unsigned long long>(o.frame_id), o.time,
                                              pb.get(), static_cast<unsigned long long>(pid), pt, now));
    const char *data = PyUnicode_AsUTF8(result.get());
    if (!data)
        throw std::runtime_error("invalid legacy result");
    cv::FileStorage json(data,
                         cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
    HybridEvidence e;
    auto z = json["zebra"];
    if (!z.empty()) {
        o.zebra_detected = int(z["detected"]) != 0;
        o.zebra = o.zebra_detected && !z["front_m"].empty() && double(z["front_m"]) >= 0;
        o.zebra_y = o.zebra ? c.front_extent + double(z["front_m"]) : 0;
        o.zebra_range_status = std::string(z["status"]);
        o.zebra_distance_reason = "legacy near-edge profile: " + o.zebra_range_status;
        if (!int(z["available"]))
            o.frame_ok = false;
        if (!z["trigger"].empty())
            o.zebra_front_trigger = double(z["trigger"]);
        e.stripes = int(z["stripes"]);
        e.near_px = z["near"].empty() ? -1. : double(z["near"]);
        e.zebra_camera = std::string(z["source"]);
        o.zebra_near_px=e.near_px;
        o.zebra_source=e.zebra_camera;
    }
    auto s = json["signal"];
    if (!s.empty()) {
        o.red = int(s["red"]) != 0;
        o.green = int(s["green"]) != 0;
        e.green_until = double(s["green_until"]);
        e.signal_stop_until = double(s["stop_until"]);
        e.signal_span_px = double(s["span_px"]);
        o.signal_stop = int(s["stop"]) != 0;
        e.signal_color = std::string(s["raw"]);
        e.signal_stop_reason = std::string(s["stop_reason"]);
        e.signal_observation = std::string(s["observation"]);
        if (!int(s["available"]))
            o.frame_ok = false;
    }
    auto park = json["parking"];
    if (!park.empty())
        o.outdoor_parking = OutdoorParkingDrive{
            int(park["duty"]), int(park["steer"]), int(park["brake"]) != 0,
            int(park["done"]) != 0, double(park["captured_s"]),
            std::string(park["stage"]), std::string(park["side"]), std::string(park["reason"])};
    auto cone = json["cone"];
    if (!cone.empty()) {
        o.cone_drive =
            LegacyConeDrive{int(cone["pwm"]),       int(cone["steering"]),      int(cone["passed"]),
                            int(cone["done"]) != 0, double(cone["captured_s"]), std::string(cone["reason"])};
        o.cone_drive->repeat_forward = int(cone["repeat_forward"]) != 0;
        e.cones_visible = int(cone["visible"]);
        e.cone_lane_valid = int(cone["lane_valid"]) != 0;
        e.cone_raw_primary = int(cone["raw_primary"]);
        e.cone_raw_secondary = int(cone["raw_secondary"]);
        e.cone_confirmed_primary = int(cone["confirmed_primary"]);
        e.cone_confirmed_secondary = int(cone["confirmed_secondary"]);
        e.cone_processing_ms = double(cone["processing_ms"]);
        e.cone_phase = std::string(cone["phase"]);
        e.cone_camera = std::string(cone["source"]);
    }
    return e;
}
} // namespace smartcar
