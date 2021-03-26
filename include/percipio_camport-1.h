/*
* Percipio All Rights Reserved 2016
*/

/**
 * @file percipio_camport.h
 * @brief depth camera interface
 *
 */


#ifndef PERCIPIO_CAMPORT_H_
#define PERCIPIO_CAMPORT_H_
#include <cassert>
#include <fstream>
#include <stdint.h>

#ifdef _WIN32

#ifdef PERCIPIO_API_EXPORTS
#define PERCIPIO_API __declspec(dllexport)
#else
#define PERCIPIO_API __declspec(dllimport)
#endif

#else

#define PERCIPIO_API __attribute__ ((visibility ("default")))

#endif

#define PERCIPIO_CAMPORT_LIB_BUILD_VERSION 3

/**
 * @brief namespace used by sdk of percipio depth camera
 */
namespace percipio {

/**
* @brief flags indicate communication mode.
*/
enum HardwareModel {
  MODEL_DPB03GN = 2,       /**< DPB03 hardware model series*/
  MODEL_DPB04GN = 3,       /**< DPB04 hadware model series*/
};


/**
* @brief data type of frame which retrieve from depth camera.
*/
enum CameraFrameDataTypes {
  CAMDATA_LEFT  = 0x00,   /**< ir data retrieve from left sensor*/
  CAMDATA_RIGHT = 0x01,   /**< ir data retrieve from right sensor*/
  CAMDATA_COLOR = 0x02,   /**< rgb data retrieve from rgb sensor*/
  CAMDATA_DEPTH = 0x03,   /**< depth data*/
  CAMDATA_POINT3D = 0x80  /**< point cloud data*/
};

/**
* @brief return values of fucntion call.
*/
enum CameraSourceStatus {
  CAMSTATUS_SUCCESS = 0, /**<  call successful*/
  CAMSTATUS_ERROR = -1,  /**<  call failed*/
  CAMSTATUS_PARAM_INVALID = -2, /**< call failed because parameter is invalid*/
  CAMSTATUS_NODATA = -3, /**< call failed because no data to be received*/
  CAMSTATUS_NOTSUPPORT = -4, /**< call failed because the feature does not support*/
  CAMSTATUS_NOTCONNECTED = -5, /**< call failed because device is not connected*/
};

/**
* @brief struct for pixel of  point cloud
*/
struct Vect3f {
  Vect3f() {}
  Vect3f(float _x, float _y, float _z) {
    x = _x;
    y = _y;
    z = _z;
  }
  float x, y, z;
};

/**
* @brief struct for buffer of image retrieve from device
* @note The image buffer is managed by this sdk. Do not need to release
* buffer manually.
*
* */
struct ImageBuffer {
  /**
   * @brief  pixel types used by different image types
   * type name format is PIX_nCm
   *  - n represents n bits per pxiel
   *  - m represents  m channel number per pixel
   * */
  enum PixelTypes {
    PIX_8C1  = 0,
    PIX_8C3  = 1,
    PIX_16C1 = 2,
    PIX_32C1 = 3,
    PIX_32FC3 = 4,
  };

  int width; /**< width of the image buffer*/
  int height;/**< height of the image buffer*/
  PixelTypes type; /**< pixel type */
  unsigned char *data; /**< pixel data buffer pointer*/

  ImageBuffer() {
    width = height = 0;
    data = 0;
  }

  /**
   * @brief Get a casted pointer at a certain row start position
   * @param row index of the row
   *
   * @par example
   * @code
   *   // get row 3 pointer in a IR image :
   *   unsigned char * p1 = buffer.ptr<unsigned char>(3);
   *   // access pixels in this row:
   *   for(int col = 0 ; col < buffer.width; col++){
   *        unsigned char pixel_value = p1[col];
   *   }
   *   //get row 1 in a cloud point buffer:
   *    Vect3f* p2 = cloud.ptr<Vect3f>(1);
   *   // access pixels in this row:
   *   for(int col = 0 ; col < cloud.width; col++){
   *        Vect3f pixel_value = p2[col];
   *   }
   * @endcode
   * */
  template <class T>
  T* ptr(int row) {
    unsigned char *t = data + get_pixel_size() * width * row;
    return (T*)t;
  }

  /**
   * @brief Get a casted pointer at a certain row start position
   * @see T* ptr(int row)
   */
  template <class T>
  const T* ptr(int row) const {
    unsigned char *t = data + get_pixel_size() * width * row;
    return (const T*)t;
  }

  /**
  * @brief get the pxiel size in byte of this image buffer
  * @return return -1 when failed
  */
  int get_pixel_size() const {
    switch (type) {
    case percipio::ImageBuffer::PIX_8C1:
      return 1;
      break;
    case percipio::ImageBuffer::PIX_8C3:
      return 3;
      break;
    case percipio::ImageBuffer::PIX_16C1:
      return 2;
      break;
    case percipio::ImageBuffer::PIX_32C1:
      return 4;
      break;
    case percipio::ImageBuffer::PIX_32FC3:
      return 4 * 3;
      break;
    default:
      return -1;
      break;
    }
  }
};

/* *
* @brief available properties of device
*
* */
enum DeviceProperties {
  PROP_DEVICE_INFO = 0x00,      /**< basic information contain name major minor hardware model*/
  PROP_DEVICE_ADDR,             /**< device address information, not support now*/
  PROP_CMOS_REG,                /**< cmos register information*/
  PROP_LASER_POW,               /**< laser power*/
  PROP_WORKMODE,                /**< work mode of device*/
  PROP_DEPTH_RESOLUTION,        /**< the resolution of depth image*/
  PROP_CALIB_DEPTH_INTRISTIC,   /**< paremeter of camera*/
  PROP_POINT_CLOUD_OUTPUT,      /**< indicates point cloud output is enabled*/
  PROP_FRAME_READY_CALLBACK,    /**< callback function when frame is ready*/
  PROP_CALLBACK_USER_DATA,      /**< data for callback function*/
  PROP_WAIT_NEXTFRAME_TIMEOUT,  /**< timeout value when wait next frame*/
  PROP_SPECKLE_FILTER,          /**< parameter of speckle filter*/
  PROP_UNDISTORT_IR,            /**< IR image undistortion , bool type , true to enable false to disable*/
  PROP_COORDINATE_MAP,          /**< get coordinate mapper*/
  PROP_TRIGGER_MODE,            /**< trigger mode */
  PROP_AUTO_GAIN_CTRL,          /**< camera auto gain control*/
};

/** @brief property types 
*
*/
typedef enum{
  PROP_TYPE_INT8,
  PROP_TYPE_INT16,
  PROP_TYPE_INT32,
  PROP_TYPE_BOOL,
  PROP_TYPE_FLOAT,
  PROP_TYPE_DOUBLE,
  PROP_TYPE_STRING,
  PROP_TYPE_STRUCT,
  PROP_TYPE_ENUM,
  PROP_TYPE_OTHER,
} PropertyDataTypes;

/**
 * @brief resolution mode list
 *
 */
typedef enum {
  RESO_MODE_160x120 = 0x00,    /**< 160x120*/
  RESO_MODE_320x240 = 0x01,    /**< 320x240*/
  RESO_MODE_640x480 = 0x02,    /**< 640x480*/
  RESO_MODE_1280x960 = 0xff,   /**< 1280x960*/
} ResolutionModes;

/**
* @brief work mode list of device
* */
typedef enum {
  WORKMODE_IDLE     = 0,       /**< initialized mode after power on*/
  WORKMODE_IR       = 1,       /**< ir mode which output ir image*/
  WORKMODE_DEPTH    = 2,       /**< depth mode which output depth image*/
  WORKMODE_IR_DEPTH = 3,       /**< ir depth mode which simultaneous output ir image*/
  WORKMODE_RGB      = 4,       /**< rgb mode which output colour image*/
  WORKMODE_RGBD     = 6,       /**< rgb mode which output colour depth image*/
  WORKMODE_TEST     = 8,       /**< test mode which only for test */
} DeviceWorkModes;

/**
* @brief log levels for library.
*
* five levels to control how much logging information output from library,
* LOG_LEVEL_WARN will output most information and LOG_LEVEL_NONE will output none.
* */
typedef enum {
  LOG_LEVEL_VERBOSE = 10,
  LOG_LEVEL_INFO = 20,
  LOG_LEVEL_WARN = 30,
  LOG_LEVEL_ERROR = 40,
  LOG_LEVEL_NONE  = 0xffff
} LogLevels;

/**
* @brief struct for setting cmos parameter.
* */
struct CmosCtrlParam {
  /**
   * camera cmos register address
   */
  int32_t reg_addr;
  /**
   * register value
   */
  int32_t reg_value;
  /** @brief camera index
  *   - 0x00 left camera
  *   - 0x01 right camera
  *   - 0x02 color camera
  *   - 0xff all camera
  */
  int8_t cam_id;
};

/**
* @brief struct for setting Camera Auto Gain Control
* @see DepthCameraDevice::SetCamGainControl()
* */
struct CamGainCtrlParam {
  CamGainCtrlParam() {}
  CamGainCtrlParam(int32_t _id, int32_t _gain) {
    cam_id = _id;
    gain = _gain;
  }
  int32_t cam_id; /**< camera id*/
  int32_t gain; /**< gain value , set to -1 for auto gain control,positive value for manual control */
};


/**
 * @brief struct for device information
 **/
struct DeviceInfo {
  int32_t hardware_model; /**< device model id*/
  uint16_t major; /**< device major version */
  uint16_t minor; /**< device minor version */
  uint8_t  sn[32]; /**< device serial number*/
  int8_t  str_name[64]; /**< device name */
};

/* *
* @brief struct for depth camera intristics parameter
* */
struct CamIntristic {
  uint32_t width;   /**< sensor pixel width*/
  uint32_t height;  /**< sensor pixel height*/
  float data[9];    /**< 3x3 intristics parameter*/
};

/**
* @brief struct for speckle filter parameter.
* set parameter to negative value will disable filter.
* */
struct SpeckleFilterParam {
  SpeckleFilterParam() {}
  SpeckleFilterParam(int max_size, int max_diff) {
    max_speckle_size = max_size;
    max_speckle_diff = max_diff;
  }
  int max_speckle_size; /**< blob size smaller than this value will be removed (default :150)*/
  int max_speckle_diff; /**<  Maximum difference between neighbor disparity pixels to put them into the same speckle blob (default:64)*/
};

/**
* @brief struct for return value of GetDeviceList()
* @see DepthCameraDevice::GetPropertyList(DeviceProperty *device_prop)
* */
typedef struct {
  uint32_t prop_id;
  char *name;
  PropertyDataTypes type;

} DeviceProperty;

/**
 * @brief callback function type
 */
typedef void(*EventCallbackFunc)(void *user_data);

/**
*@brief coordinate convert
*/
class ICoordinateMapper {
 public:
  virtual ~ICoordinateMapper() {}
  virtual const CamIntristic* get_depth_intristics() = 0;
  /**
  * convert point on depth image to world coordinate
  *@param[in] depth_position  depth image position .
  *@param[out]   world_position point cloud.
  */
  virtual int DepthToWorld(const Vect3f *depth_position, Vect3f *world_position) = 0;
  /**
  * convert depth map to cloud points
  *@param[in] depth  depth map .Buffer type should be PIX_16C1.
  *@param[out] cloud  point cloud .buffer type is PIX_32FC3.
  */
  virtual int DepthToWorld(const ImageBuffer *depth, ImageBuffer *cloud) = 0;
  virtual int WorldToDepth(const Vect3f *world_position, Vect3f *depth_position) = 0;
};

/**
 * @brief device abi interface
 **/
class ICameraVideoSource {
 public:
  virtual ~ICameraVideoSource() {}
  virtual int GetDeviceNum() = 0;
  virtual int GetPropertyNum() = 0;
  virtual int GetDeviceList(int *devs) = 0;
  virtual int GetPropertyList(DeviceProperty *device_prop) = 0;
  //trigger device to get a frame sync
  virtual CameraSourceStatus FramePackageGetSync() = 0;
  //trigger device to get a frame async
  virtual CameraSourceStatus FramePackageGetAsync() = 0;
  virtual CameraSourceStatus FramePackageGet() = 0;
  virtual CameraSourceStatus FrameGet(int cam_data_type, ImageBuffer *buff1) = 0;
  virtual CameraSourceStatus OpenDevice(int id) = 0;
  virtual CameraSourceStatus OpenDevice() = 0;
  virtual void CloseDevice() = 0;
  virtual CameraSourceStatus Config(const char *data) = 0;
  /**@return status
   *  - negative value for error status
   *  - positive value for the actual data size
   */
  virtual int GetProperty(int prop_id, void *data_buff, int size) = 0;
  /**@return status
   *  - negative value for error status
   *  - positive value for the actual data size
   */
  virtual int SetProperty(int prop_id, const void *data, int size) = 0;
};

//create camera data source
PERCIPIO_API ICameraVideoSource* CreateSource(HardwareModel model);
PERCIPIO_API void ReleaseSource(ICameraVideoSource* ptr);

/**
* @brief the log display level of library.
* @param level is a integer value.
* @see enum LogLevels.
*/
PERCIPIO_API  void SetLogLevel(int level);

/**
* @brief the version of library.
*/
PERCIPIO_API  int LibVersion();


/**
* @brief this class represent a depth camera device.
*
* In percipio sdk, the main APIs provided by member functions of this class,
* in application program, firstly, create a instance of this class, then all
* operations on the depth camera device can performed by this instance.
*/
class DepthCameraDevice {
 public:
  /**
   * @brief this is a constructor
   * @param model is an enumeration type
   */
  DepthCameraDevice(HardwareModel model = MODEL_DPB04GN) {
    assert(LibVersion() == PERCIPIO_CAMPORT_LIB_BUILD_VERSION);
    _source = CreateSource(model);
  }

  /**
   * @brief this is a destructor
  */
  ~DepthCameraDevice() {
    ReleaseSource(_source);
  }

  /**
   * @brief create an ICameraVideoSource instance according to model,
   * which associated with a real device.
   * @param model an enumeration model type.
   */
  void Create(HardwareModel model = MODEL_DPB04GN) {
    if (_source) {
      ReleaseSource(_source);
      _source = NULL;
    }
    _source = CreateSource(model);
  }

  /**
   * @brief get ICameraVideoSource interface related to this camera device.
   * @see function Create().
   */
  ICameraVideoSource * get_source() const {
    return _source;
  }

  /**
   * @brief retrieve property list of the device.
   * @param device_prop is a struct type.
   * @see struct DeviceProperty.
   */
  int GetPropertyList(DeviceProperty *device_prop) {
    return get_source()->GetPropertyList(device_prop);
  }

  /**
   * @brief retrieve available device list on current machine.
   * @param devs is a integer array.
   */
  int GetDeviceList(int *devs) {
    return get_source()->GetDeviceList(devs);
  }

  /**
   * @brief retrieve the number of devices on current machine.
   */
  int GetDeviceNum() {
    return get_source()->GetDeviceNum();
  }

  /**
   * @brief retrieve the number of properties.
   */
  int GetPropertyNum() {
    return get_source()->GetPropertyNum();
  }

  /**
   * @brief retrieve frames captured by device on the same moment.
   *
   * a frame package represents a set of image frames captured at the same time.
   * The frames will save in the internal buffer which managed by this library.
   * In different work mode, the number of frames may be one/two/three, and
   * to get one frame, using FrameGet().
   * @see function FrameGet().
   **/
  CameraSourceStatus FramePackageGet() {
    return get_source()->FramePackageGet();
  }

  /**
   * @brief retrieve on frame from internal buffer which updated by FramePackageGet().
   *
   * @note Frame data in current buffer won't be modified until FramePackageGet , FramePackageGetSync or FramePackageGetASync is called
   * @param cam_id is the frame data type which descriped by emnu CameraFrameDataTypes.
   * @param buff is a ImageBuffer buffer which used to save frame data.
   * @see enum CameraFrameDataTypes.
   * @see struct ImageBuffer.
   **/
  CameraSourceStatus FrameGet(int cam_id, ImageBuffer *buff) {
    return get_source()->FrameGet(cam_id, buff);
  }

  /**
   * @brief Retrieve frame package and wait until received in trigger mode.
   *
   * call this method will trigger device to capture one frame package and wait unitl data is ready to read.
   * this method will return with error code if time out.
   * @see FrameGet()
   * @see SetWaitNextFrameTimeout()
   */
  CameraSourceStatus FramePackageGetSync() {
    return get_source()->FramePackageGetSync();
  }

  /**
   * @brief retrieve frame package, return immediately in trigger mode.
   *
   * call this method will trigger device to capture one frame package and return immediately.
   */
  CameraSourceStatus FramePackageGetAsync() {
    return get_source()->FramePackageGetAsync();
  }

  /**
   * @brief open device according id to which get from GetDeviceList().
   * @param id is an integer value.
   * @see function GetDeviceList().
   **/
  CameraSourceStatus OpenDevice(int id) {
    return get_source()->OpenDevice(id);
  }

  /**
   * @brief open default device, equal to OpenDevice(1).
   */
  CameraSourceStatus OpenDevice() {
    return get_source()->OpenDevice();
  }

  /**
   * @brief close current device.
   */
  void CloseDevice() {
    get_source()->CloseDevice();
  }

  /**
   * @brief load device parameter from string
   *
   * this method config device parameter with a xml format string data.
   * @param[in] data the c string which contain deivce parameter
   *
   */
  void LoadConfig(const char *data) {
    get_source()->Config(data);
  }

  /**
   * @brief load device parameter from a file
   * @param[in] filename is the path of configuration file.
   * @see LoadConfig()
   */
  CameraSourceStatus LoadConfigFile(const char *filename) {
    std::ifstream ifs(filename);
    if (ifs) {
      std::string str((std::istreambuf_iterator<char>(ifs)),
                      std::istreambuf_iterator<char>());
      return get_source()->Config(str.c_str());
    }
    return CAMSTATUS_PARAM_INVALID;
  }

  /**
   * @brief retrieve property values.
   * @param[in] prop_id the unique identification for a property.
   * @param[out] data value of property.
   * @param[in] size the size of parameter data.
   * @return status or data size
   *     - negative value when failed
   *     - size of data when success
   * @see enum DeviceProperties.
   */
  int GetProperty(int prop_id, void *data, int size) {
    return get_source()->GetProperty(prop_id, (char*)data, size);
  }

  /**
   * @brief set integer value for the property of device.
   * @param prop_id is the unique identification for a property.
   * @param data value to be set, interge type.
   * @return status or data size
   *     - negative value when failed
   *     - size of data when success
   */
  int SetProperty_Int(int prop_id, int data) {
    return get_source()->SetProperty(prop_id, &data, sizeof(data));
  }

  /**
   * @brief set boolean value for the property of device.
   * @param prop_id is the unique identification for a property.
   * @param data value to be set, boolean type.
   * @return status or data size
   *     - negative value when failed
   *     - size of data when success
   */
  int SetProperty_Bool(int prop_id, bool data) {
    return get_source()->SetProperty(prop_id, &data, sizeof(data));
  }

  /**
   * @brief set string value for the property of device.
   * @param prop_id is the unique identification for a property.
   * @param data value to be set, string type.
   * @return status or data size
   *     - negative value when failed
   *     - size of this pointer when success
   */
  int SetProperty_String(int prop_id, char *data) {
    return get_source()->SetProperty(prop_id, data, sizeof(data));
  }

  /**
   * @brief set value for cmos register.
   * @param camid is an integer argument.
            - 0x00 left sensor
            - 0x01 right sensor
            - 0xff all sensor
   * @param regid is the address of register
   * @param value integer value to be set.
   */
  int SetCmosRegister(int camid, int regid, int value) {
    CmosCtrlParam param;
    param.cam_id = camid;
    param.reg_addr = regid;
    param.reg_value = value;
    return get_source()->SetProperty(PROP_CMOS_REG, &param, sizeof(param));
  }

  /**
   * @brief set laser power, equal to SetProperty_Bool(PROP_LASER_POW, ...).
   * @param [in] duty bright of laser .
           - 100 full power.
           - 0   off
   * @return status or data size
   *     - negative value when failed
   *     - size of data when success
   */
  int SetLaser(unsigned char duty) {
    return get_source()->SetProperty(PROP_LASER_POW, &duty, sizeof(duty));
  }

  /**
   * @brief set output resolution.
   * @param reso struct ResolutionModes type value.
   * @return status or data size
   *     - negative value when failed
   *     - size of data when success
   * @see struct ResolutionModes.
   */
  int SetDepthResolution(ResolutionModes reso) {
    return get_source()->SetProperty(PROP_DEPTH_RESOLUTION, &reso, sizeof(reso));
  }

  /**
   * @brief set work mode of device, equal to SetProperty_Int(percipio::PROP_WORKMODE, ...).
   * @return status or data size
   *     - negative value when failed
   *     - size of data when success
   * @param mode struct DeviceWorkModes type value.
   * @see struct DeviceWorkModes.
   * @see function SetProperty_Int().
   */
  int SetWorkMode(DeviceWorkModes mode) {
    return get_source()->SetProperty(percipio::PROP_WORKMODE, &mode, sizeof(mode));
  }

  /**
   * @brief enable/disable point cloud output, equal to SetProperty_Bool(percipio::PROP_POINT_CLOUD_OUTPUT, ...),
   *
   * compute Point cloud for each frame, this require WORKMODE_DEPTH or WORKMODE_IR_DEPTH.
   * @return status or data size
   *     - negative value when failed
   *     - size of data when success
   * @param[in] enable boolean value.
   * @see function SetProperty_Bool();
   */
  int SetPointCloudOutput(bool enable) {
    return get_source()->SetProperty(percipio::PROP_POINT_CLOUD_OUTPUT, &enable, sizeof(enable));
  }

  /**
   * @brief set call back function when one frame is received.
   * @param[in] callback function pointer.
   * @return status or data size
   *     - negative value when failed
   *     - size of data when success
   */
  int SetFrameReadyCallback(EventCallbackFunc callback) {
    return get_source()->SetProperty(percipio::PROP_FRAME_READY_CALLBACK, (void *)callback, sizeof(callback));
  }

  /**
   * @brief set callback function data.
   * @param data pointer to data.
   * @return status or data size
   *     - negative value when failed
   *     - size of data when success
   * @see function SetFrameReadyCallback().
   */
  int SetCallbackUserData(void* data) {
    return get_source()->SetProperty(percipio::PROP_CALLBACK_USER_DATA, data, sizeof(data));
  }

  /**
   * @brief set max blocking time when call FramePackageGet(),
   *
   * invoking thread will be blocked until new frame arrived or time out.
   * for not-blocking FramePackageGet() call, set value to zero or negative.
   * @param[in] timeout_ms block waiting time in milliseconds.
   * @return status or data size
   *     - negative value when failed
   *     - size of data when success
   */
  int SetWaitNextFrameTimeout(int timeout_ms) {
    return get_source()->SetProperty(percipio::PROP_WAIT_NEXTFRAME_TIMEOUT, &timeout_ms, sizeof(timeout_ms));
  }

  /**
   * @brief set speckle fileter parameter
   * @param param a struct SpeckleFilterParam type argument
   * @return status or data size
   *     - negative value when failed
   *     - size of data when success
   * @see struct SpeckleFilterParam
   */
  int SetSpeckleFilter(const SpeckleFilterParam &param) {
    return get_source()->SetProperty(percipio::PROP_SPECKLE_FILTER, &param, sizeof(param));
  }

  /**
   * @brief set camera gain status
   * @param [in] cam_id camera index
   * @param [in] gain  gain value ,-1 for auto control ,positive value for manual control.
   * @return status or data size
   *     - negative value when failed
   *     - size of data when success
   * @see struct CamGainCtrlParam
   */
  int SetCamGainControl(int cam_id, int gain) {
    CamGainCtrlParam param(cam_id, gain);
    return get_source()->SetProperty(percipio::PROP_AUTO_GAIN_CTRL, &param, sizeof(param));
  }

  /**
   * @brief get camera gain status
   * @param [in] cam_id camera index
   * @param [out] gain  gain value ,-1 for auto control ,positive value for manual control.
   * @see struct CamGainCtrlParam
   * @return status or data size
   *     - negative value when failed
   *     - size of data when success
   * @see SetCamGainControl()
   */
  int GetCamGainControl(int cam_id, int *gain) {
    CamGainCtrlParam param(cam_id, 0);
    int res = get_source()->GetProperty(percipio::PROP_AUTO_GAIN_CTRL, &param, sizeof(param));
    *gain = param.gain;
    return res;
  }

  /**
   * @brief get speckle fileter parameter
   * @param param a struct SpeckleFilterParam type argument
   * @see struct SpeckleFilterParam
   * @return status or data size
   *     - negative value when failed
   *     - size of data when success
   */
  int GetSpeckleFilter(SpeckleFilterParam *param) {
    return get_source()->GetProperty(percipio::PROP_SPECKLE_FILTER, (char*)param, sizeof(*param));
  }

  /**
  * @brief get a CoordinateMapper object
    @param [out] mapper output a ICoordinateMapper pointer.
  * @see class CoordinateMapper
  * @return status or data size
  *     - negative value when failed
  *     - size of this pointer when success
  */
  int GetCoordinateMapper(ICoordinateMapper **mapper) {
    return get_source()->GetProperty(percipio::PROP_COORDINATE_MAP, (char*)mapper, sizeof(ICoordinateMapper**));
  }


 private:
  DepthCameraDevice(const DepthCameraDevice&);
  ICameraVideoSource * _source;
};
}//namespace percipio

#endif
