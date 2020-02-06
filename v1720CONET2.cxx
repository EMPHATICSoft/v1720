/*****************************************************************************/
/**
\file v1720CONET2.cxx

## Contents

This file contains the class implementation for the v1720 module driver.
 *****************************************************************************/

#include "v1720CONET2.hxx"
#include <execinfo.h>
#include <algorithm>
#include <vector>
#include <cmath>
#include <fstream>

#define UNUSED(x) ((void)(x)) //!< Suppress compiler warnings
#define NUM_SQ_WORDS 5
#define POSTPONE_CHARGE_FILTERING 0
#define MAX_QT_WORDS 2000

//! Configuration string for this board. (ODB: /Equipment/[eq_name]/Settings/[board_name]/)
const char * v1720CONET2::config_str_board[] = {\
    "setup = INT : 0",\
    "Acq mode = INT : 3",\
    "Channel Configuration = DWORD : 16",\
    "Buffer organization = INT : 10",\
    "Custom size = INT : 625",\
    "Channel Mask = DWORD : 255",\
    "Trigger Source = DWORD : 1073741824",\
    "Trigger Output = DWORD : 1073741824",\
    "Post Trigger = DWORD : 576",\
    /*"fp_io_ctrl   = DWORD : 0x104", */ \
    "almost_full = DWORD : 512",\
    /*"fp_lvds_io_ctrl = DWORD : 0x22", */  \
    "AutoTrig_Threshold = DWORD[8] :",\
    "[0] 3870",\
    "[1] 3870",\
    "[2] 3870",\
    "[3] 3870",\
    "[4] 3870",\
    "[5] 3870",\
    "[6] 3870",\
    "[7] 3870",\
    "AutoTrig_N_4Bins_Min = DWORD[8] :",\
    "[0] 2",\
    "[1] 2",\
    "[2] 2",\
    "[3] 2",\
    "[4] 2",\
    "[5] 2",\
    "[6] 2",\
    "[7] 2",\

    "ZLESignedThresh = INT[8] :",\
    "[0] -3895",\
    "[1] -3895",\
    "[2] -3895",\
    "[3] -3895",\
    "[4] -3895",\
    "[5] -3895",\
    "[6] -3895",\
    "[7] -3895",\

    "ZLENBinsBefore = DWORD[8] :",\
    "[0] 0x5",\
    "[1] 0x5",\
    "[2] 0x5",\
    "[3] 0x5",\
    "[4] 0x5",\
    "[5] 0x5",\
    "[6] 0x5",\
    "[7] 0x5",\

    "ZLENBinsAfter = DWORD[8] :",\
    "[0] 0x5",\
    "[1] 0x5",\
    "[2] 0x5",\
    "[3] 0x5",\
    "[4] 0x5",\
    "[5] 0x5",\
    "[6] 0x5",\
    "[7] 0x5",\

    "DAC = DWORD[8] :",\
    "[0] 10000",\
    "[1] 10000",\
    "[2] 10000",\
    "[3] 10000",\
    "[4] 10000",\
    "[5] 10000",\
    "[6] 10000",\
    "[7] 10000",\

    "QT Bank = BOOL : y",\
    "Minima Bank = BOOL : y",\
    "Smart QT Bank = BOOL : n",\
    "Smart QT PulseDerivativeThresho = INT : 3",\
    "Smart QT PulseStop_MaxHeight = FLOAT : 2",\
    "Smart QT PulseStop_MaxDeriv = INT : 2",\
    "Smart QT PulseExtend_MinHeight = FLOAT : 3",\
    "Smart QT MinPulseCharge = FLOAT : 50",\
    "Smart QT MaxPulseCharge = FLOAT : 500",\
    "Smart QT MaxWidthIntercept = FLOAT : 1254",\
    "Smart QT MaxWidthSlope = FLOAT : 0.015184382",\
    NULL
};

const char v1720CONET2::history_settings[][NAME_LENGTH] = { "eStored", "busy", "rb_level" };

/**
 * \brief   Constructor for the module object
 *
 * Set the basic hardware parameters
 *
 * \param   [in]  board     Board number on the optical link
 */
#if SIMULATION
v1720CONET2::v1720CONET2(int board, HNDLE hDB) : board_(board), moduleID_(board), odb_handle_(hDB)
{
  device_handle_ = -1;
  settings_handle_ = 0;
  running_=false;
  settings_loaded_=false;
  settings_touched_=false;
  data_type_ = RawPack2;
  rb_handle_ = -1;
  num_events_in_rb_ = 0;
  verbosity_ = 0;
}
#else
/**
 * \brief   Constructor for the module object
 *
 * Set the basic hardware parameters
 *
 * \param   [in]  feindex   Frontend index number
 * \param   [in]  link      Optical link number
 * \param   [in]  board     Board number on the optical link
 * \param   [in]  moduleID  Unique ID assigned to module
 */
v1720CONET2::v1720CONET2(int feindex, int link, int board, int moduleID, HNDLE hDB)
: feIndex_(feindex), link_(link), board_(board), moduleID_(moduleID), odb_handle_(hDB), num_events_in_rb_(0)
{
  device_handle_ = -1;
  settings_handle_ = 0;
  settings_loaded_ = false;
  settings_touched_ = false;
  running_= false;
  data_type_ = RawPack2;
  rb_handle_ = -1;
  verbosity_ = 0;

  for (int i = 0; i < 32; i++) {
    for (int j = 0; j < 8; j++) {
      channel_minima_[i][j] = 4096;
    }
  }

  for (int i = 0; i < 8; i++) {
    pmts[i] = NULL;
  }

  DefaultBaseline = 3901.;
  CurrentDefaultBaseline = DefaultBaseline;
  saturationFail = false;
  baselineFail = false;
  IsAdjacent = false;

  pfConfig.ChargeCut = 50;
  pfConfig.SubPeakDerivThreshold = -3;
  pfConfig.DerivThreshold = 3;
  pfConfig.StopDeriv = 2;
  pfConfig.StopVolt = 2;
  pfConfig.MinBaselineSamples = 4;
  pfConfig.MaxBaselineSamples = 62;
  pfConfig.DoHiddenPeakFind = false;
  pfConfig.ChargeResidualLimit = 50;
  pfConfig.ClosePulseHeightThresh = 3;
  pfConfig.ClosePulseDerivThresh = 3;
  pfConfig.ClosePulseLookAhead = 3;
  pfConfig.ClosePulseNearEndOfBlock = 4;
  pfConfig.ExtendPulseEnd = true;
  pfConfig.EnableSubPeaks = false;
  pfConfig.EnablePulseSplitting = false;
  pfConfig.MaxSPEWidthIntercept = 1254;
  pfConfig.MaxSPEWidthSlope = 0.015184382;
  pfConfig.MaxSPECharge = 500;
  pfConfig.SplitPulseCharge = 65535;

  ResetForNewChannel();
  ResetForNewPulse();

  IsAdjacent = false;

  LoadSmartQTConfig(); // Read a bunch of other params from a txt file
}
/**
 * Move constructor needed because we're putting v1720CONET2 objects in a vector which requires
 * either the copy or move operation.  The implicit move constructor (or copy constructor)
 * cannot be created by the compiler because our class contains an atomic object with a
 * deleted copy constructor. */
v1720CONET2::v1720CONET2(v1720CONET2&& other) noexcept
: feIndex_(std::move(other.feIndex_)), link_(std::move(other.link_)), board_(std::move(other.board_)),
    moduleID_(std::move(other.moduleID_)), odb_handle_(std::move(other.odb_handle_)),
        num_events_in_rb_(other.num_events_in_rb_.load())
{
  device_handle_ = std::move(other.device_handle_);
  settings_handle_ = std::move(other.settings_handle_);
  settings_loaded_ = std::move(other.settings_loaded_);
  settings_touched_ = std::move(other.settings_touched_);
  running_= std::move(other.running_);
  data_type_ = std::move(other.data_type_);
  rb_handle_ = std::move(other.rb_handle_);
  data_type_ = std::move(other.data_type_);
  verbosity_ = std::move(other.verbosity_);
  config = std::move(other.config);

  for (int i = 0; i < 32; i++) {
    for (int j = 0; j < 8; j++) {
      channel_minima_[i][j] = std::move(other.channel_minima_[i][j]);
    }
  }

  for (int i = 0; i < 8; i++) {
    pmts[i] = std::move(other.pmts[i]);
  }

  DefaultBaseline = std::move(other.DefaultBaseline);
  CurrentDefaultBaseline = std::move(other.CurrentDefaultBaseline);
  saturationFail = std::move(other.saturationFail);
  baselineFail = std::move(other.baselineFail);
  IsAdjacent = std::move(other.IsAdjacent);

  pulseState = std::move(other.pulseState);
  channelState = std::move(other.channelState);
  pfConfig = std::move(other.pfConfig);
  spePdf = std::move(other.spePdf);
}
v1720CONET2& v1720CONET2::operator=(v1720CONET2&& other) noexcept
{
  if (this != &other){  //if trying to assign object to itself

    feIndex_ = std::move(other.feIndex_);
    link_ = std::move(other.link_);
    board_ = std::move(other.board_);
    moduleID_ = std::move(other.moduleID_);
    odb_handle_ = std::move(other.odb_handle_);
    num_events_in_rb_ = other.num_events_in_rb_.load();
    device_handle_ = std::move(other.device_handle_);
    settings_handle_ = std::move(other.settings_handle_);
    settings_loaded_ = std::move(other.settings_loaded_);
    settings_touched_ = std::move(other.settings_touched_);
    running_= std::move(other.running_);
    rb_handle_ = std::move(other.rb_handle_);
    data_type_ = std::move(other.data_type_);
    verbosity_ = std::move(other.verbosity_);
    config = std::move(other.config);


    for (int i = 0; i < 32; i++) {
      for (int j = 0; j < 8; j++) {
        channel_minima_[i][j] = std::move(other.channel_minima_[i][j]);
      }
    }

    for (int i = 0; i < 8; i++) {
      pmts[i] = std::move(other.pmts[i]);
    }

    DefaultBaseline = std::move(other.DefaultBaseline);
    CurrentDefaultBaseline = std::move(other.CurrentDefaultBaseline);
    saturationFail = std::move(other.saturationFail);
    baselineFail = std::move(other.baselineFail);
    IsAdjacent = std::move(other.IsAdjacent);

    pulseState = std::move(other.pulseState);
    channelState = std::move(other.channelState);
    pfConfig = std::move(other.pfConfig);
    spePdf = std::move(other.spePdf);
  }
  return *this;
}
#endif  //SIMULATION

//
//--------------------------------------------------------------------------------
/**
 * \brief   Destructor for the module object
 *
 * Nothing to do.
 */
v1720CONET2::~v1720CONET2()
{
}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Get short string identifying the module's index, link and board number
 *
 * \return  name string
 */
std::string v1720CONET2::GetName()
{
  std::stringstream txt;
#if SIMULATION
  txt << "B" << std::setfill('0') << std::setw(2) << board_;
#else
  txt << "F" << std::setfill('0') << std::setw(2) << feIndex_
      << "L" << std::setfill('0') << std::setw(2) << link_
      << "B" << std::setfill('0') << std::setw(2) << board_;
#endif
  return txt.str();
}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Get connected status
 *
 * \return  true if board is connected
 */
bool v1720CONET2::IsConnected()
{
  return (device_handle_ >= 0);
}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Get run status
 *
 * \return  true if run is started
 */
bool v1720CONET2::IsRunning()
{
  return running_;
}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Connect the board through the optical link
 *
 * \return  CAENComm Error Code (see CAENComm.h)
 */
v1720CONET2::ConnectErrorCode v1720CONET2::Connect()
{
  return Connect(2, 5);  //reasonable default values
}

//
//--------------------------------------------------------------------------------
v1720CONET2::ConnectErrorCode v1720CONET2::Connect(int connAttemptsMax, int secondsBeforeTimeout)
{
  if (verbosity_) std::cout << GetName() << "::Connect()\n";

  ConnectErrorCode returnCode;

  if (IsConnected()) {
    cm_msg(MERROR,"Connect","Board %d already connected", this->GetModuleID());
    returnCode = ConnectErrorAlreadyConnected;
  }

#if SIMULATION
  if (verbosity_) std::cout << "Opening device " << moduleID_ << std::endl;

  device_handle_ = 1;  //random
  printf("Connected successfully to module; handle: %d\n",this->GetDeviceHandle());

#else

  /* The optical connection hangs quite often, do it in a thread and timeout if necessary */
  CAENComm_ErrorCode sCAEN;
  pthread_mutex_t m = PTHREAD_MUTEX_INITIALIZER;
  pthread_cond_t cv = PTHREAD_COND_INITIALIZER;

  pthread_t con_thread;
  volatile struct v1720CONET2::thread_args con_thread_args = { this , &sCAEN, &cv };
  timespec wait_time;
  wait_time.tv_nsec = 0;
  int status;

  /* For info purposes */
  std::stringstream * ssErrMsg;
  timespec start_time, end_time;

  start_time.tv_nsec = end_time.tv_nsec = 0;
  start_time.tv_sec = time(NULL);

  for (int i = 0; i < connAttemptsMax; ++i){

    status = pthread_create(&con_thread, NULL, &v1720CONET2::connectThread, (void*)&con_thread_args);
    if(status){
      cm_msg(MERROR,"Connect", "Couldn't create thread for link %d board %d. Return code: %d",
          this->GetLink(), this->GetBoard(), status);
    }
    pthread_mutex_lock(&m);
    ssErrMsg = new std::stringstream;

    wait_time.tv_sec = time(NULL) + secondsBeforeTimeout;
    if(pthread_cond_timedwait(&cv, &m, &wait_time) == ETIMEDOUT){
      end_time.tv_sec = time(NULL);

      pthread_mutex_unlock(&m);

      *ssErrMsg << "CAENComm_OpenDevice attempt #" << i+1 << " timeout (" << secondsBeforeTimeout << "s).";
      *ssErrMsg << " Total elapsed time: " << end_time.tv_sec - start_time.tv_sec << "s";
      *ssErrMsg << " FE Index: " << feIndex_;
      *ssErrMsg << " Link: " << link_;
      *ssErrMsg << " Board: " << board_;
      *ssErrMsg << " Module ID: " << moduleID_;

      if(i < (connAttemptsMax - 1)){
        *ssErrMsg << " Retrying... ";
      }

      cm_msg(MERROR, "Connect", ssErrMsg->str().c_str());
      returnCode = ConnectErrorTimeout;
    }
    else{
      end_time.tv_sec = time(NULL);

      pthread_mutex_unlock(&m);
      pthread_detach(con_thread);

      if (sCAEN == CAENComm_Success) {
        printf("Link#:%d Board#:%d Module_Handle[%d]:%d\n",
                link_, board_, moduleID_, this->GetDeviceHandle());

        returnCode = ConnectSuccess;
      }
      else {
        device_handle_ = -1;

        *ssErrMsg << "CAENComm_OpenDevice error.";
        *ssErrMsg << " FE Index: " << feIndex_;
        *ssErrMsg << " Link: " << link_;
        *ssErrMsg << " Board: " << board_;
        *ssErrMsg << " Module ID: " << moduleID_;
        *ssErrMsg << " CAENComm_ErrorCode: " << sCAEN;

        cm_msg(MERROR, "Connect", ssErrMsg->str().c_str());
        returnCode = ConnectErrorCaenComm;
      }

      break;
    }
  }

#endif
  return returnCode;
}

//
//--------------------------------------------------------------------------------
void * v1720CONET2::connectThread(void * arg){

  v1720CONET2::thread_args * t_args = (v1720CONET2::thread_args*)arg;

  std::cout << "Opening device (i,l,b) = ("
      << t_args->v1720->feIndex_ << ","
      << t_args->v1720->link_ << ","
      << t_args->v1720->board_ << ")" << std::endl;

  *(t_args->errcode) = CAENComm_OpenDevice(CAENComm_PCIE_OpticalLink, t_args->v1720->link_, t_args->v1720->board_,
      0, &(t_args->v1720->device_handle_));
  pthread_cond_signal(t_args->cv);

  return NULL;
}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Disconnect the board through the optical link
 *
 * \return  CAENComm Error Code (see CAENComm.h)
 */
bool v1720CONET2::Disconnect()
{
  if (verbosity_) std::cout << GetName() << "::Disconnect()\n";

  if (!IsConnected()) {
    cm_msg(MERROR,"Disconnect","Board %d already disconnected", this->GetModuleID());
    return false;
  }
  if (IsRunning()) {
    cm_msg(MERROR,"Disconnect","Can't disconnect board %d: run in progress", this->GetModuleID());
    return false;
  }

#if SIMULATION
  if (verbosity_) std::cout << "Closing device " << moduleID_ << std::endl;

  CAENComm_ErrorCode sCAEN = CAENComm_Success;
#else
  if (verbosity_) std::cout << "Closing device (i,l,b) = (" << feIndex_ << "," << link_ << "," << board_ << ")" << std::endl;

  CAENComm_ErrorCode sCAEN = CAENComm_CloseDevice(device_handle_);
#endif

  if(sCAEN == CAENComm_Success){
    device_handle_ = -1;
  }
  else
    return false;

  return true;
}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Start data acquisition
 *
 * Write to Acquisition Control reg to put board in RUN mode. If ODB
 * settings have been changed, re-initialize the board with the new settings.
 * Set _running flag true.
 *
 * \return  CAENComm Error Code (see CAENComm.h)
 */
bool v1720CONET2::StartRun()
{
  if (verbosity_) std::cout << GetName() << "::StartRun()\n";

  if (IsRunning()) {
    cm_msg(MERROR,"StartRun","Board %d already started", this->GetModuleID());
    return false;
  }
  if (!IsConnected()) {
    cm_msg(MERROR,"StartRun","Board %d disconnected", this->GetModuleID());
    return false;
  }

	//  if (settings_touched_)
	if(1) // Always reset the parameters
  {
		//    cm_msg(MINFO, "StartRun", "Note: settings on board %s touched. Re-initializing board.",
		//  GetName().c_str());
    std::cout << "reinitializing" << std::endl;

    //Re-read the record from ODB, it may have changed
    int size = sizeof(V1720_CONFIG_SETTINGS);
    db_get_record(odb_handle_, settings_handle_, &config, &size, 0);

    // So we can re-use exactly the same code as RAT for pulse-finding,
    // we need to copy some parameters over

    pfConfig.ChargeCut = config.MinPulseCharge; // 50
    pfConfig.DerivThreshold = config.PulseDerivativeThreshold; // 3
    pfConfig.StopDeriv = config.PulseStop_MaxDeriv; // 2
    pfConfig.StopVolt = config.PulseStop_MaxHeight; // 2
    pfConfig.ClosePulseHeightThresh = config.PulseExtend_MinHeight; //3
    pfConfig.MaxSPEWidthIntercept = config.MaxWidthIntercept; // 1254
    pfConfig.MaxSPEWidthSlope = config.MaxWidthSlope; // 0.015184382
    pfConfig.MaxSPECharge = config.MaxPulseCharge; // 500
		
    int status = InitializeForAcq();
		if (status == -1) return false;
  }
	
  CAENComm_ErrorCode e = AcqCtl_(V1720_RUN_START);
  if (e == CAENComm_Success)
    running_=true;
  else
    return false;
	
  return true;
}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Start data acquisition
 *
 * Write to Acquisition Control reg to put board in STOP mode.
 * Set _running flag false.
 *
 * \return  CAENComm Error Code (see CAENComm.h)
 */
bool v1720CONET2::StopRun()
{
  if (verbosity_) std::cout << GetName() << "::StopRun()\n";

  if (!IsRunning()) {
    cm_msg(MERROR,"StopRun","Board %d already stopped", this->GetModuleID());
    return false;
  }
  if (!IsConnected()) {
    cm_msg(MERROR,"StopRun","Board %d disconnected", this->GetModuleID());
    return false;
  }

  CAENComm_ErrorCode e = AcqCtl_(V1720_RUN_STOP);
  if (e == CAENComm_Success)
    running_=false;
  else
    return false;

  return true;
}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Setup board registers using preset (see ov1720.c:ov1720_Setup())
 *
 * Setup board registers using a preset defined in the midas file ov1720.c
 * - Mode 0x0: "Setup Skip\n"
 * - Mode 0x1: "Trigger from FP, 8ch, 1Ks, postTrigger 800\n"
 * - Mode 0x2: "Trigger from LEMO\n"
 *
 * \param   [in]  mode Configuration mode number
 * \return  CAENComm Error Code (see CAENComm.h)
 */
CAENComm_ErrorCode v1720CONET2::SetupPreset_(int mode)
{
#if SIMULATION
  return CAENComm_Success;
#else
  return ov1720_Setup(device_handle_, mode);
#endif
}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Control data acquisition
 *
 * Write to Acquisition Control reg
 *
 * \param   [in]  operation acquisition mode (see v1720.h)
 * \return  CAENComm Error Code (see CAENComm.h)
 */
CAENComm_ErrorCode v1720CONET2::AcqCtl_(uint32_t operation)
{
#if SIMULATION
  return CAENComm_Success;
#else

//  Obsolete, we need LVDS BusyIn bit (8)
//  return ov1720_AcqCtl(_device_handle, operation);

  uint32_t reg;
  CAENComm_ErrorCode sCAEN;

  sCAEN = CAENComm_Read32(device_handle_, V1720_ACQUISITION_CONTROL, &reg);

  switch (operation) {
  case V1720_RUN_START:
                printf("Reg: 0x%x\n", V1720_ACQUISITION_CONTROL);
    sCAEN = CAENComm_Write32(device_handle_, V1720_ACQUISITION_CONTROL, (reg | 0x4));
    break;
  case V1720_RUN_STOP:
    sCAEN = CAENComm_Write32(device_handle_, V1720_ACQUISITION_CONTROL, (reg & ~( 0x4)));

    break;
  case V1720_REGISTER_RUN_MODE:
    sCAEN = CAENComm_Write32(device_handle_, V1720_ACQUISITION_CONTROL, 0x100);
    break;
  case V1720_SIN_RUN_MODE:
    sCAEN = CAENComm_Write32(device_handle_, V1720_ACQUISITION_CONTROL, 0x101);
    break;
  case V1720_SIN_GATE_RUN_MODE:
    sCAEN = CAENComm_Write32(device_handle_, V1720_ACQUISITION_CONTROL, 0x102);
    break;
  case V1720_MULTI_BOARD_SYNC_MODE:
    sCAEN = CAENComm_Write32(device_handle_, V1720_ACQUISITION_CONTROL, 0x103);
    break;
  case V1720_COUNT_ACCEPTED_TRIGGER:
    sCAEN = CAENComm_Write32(device_handle_, V1720_ACQUISITION_CONTROL, (reg & ~( 0x8)));
    break;
  case V1720_COUNT_ALL_TRIGGER:
    sCAEN = CAENComm_Write32(device_handle_, V1720_ACQUISITION_CONTROL, (reg | 0x8));
    break;
  default:
    printf("operation %d not defined\n", operation);
    break;
  }
  return sCAEN;


#endif
}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Control data acquisition
 *
 * Write to Acquisition Control reg
 *
 * \param   [in]  operation acquisition mode (see v1720.h)
 * \return  CAENComm Error Code (see CAENComm.h)
 */
CAENComm_ErrorCode v1720CONET2::WriteChannelConfig_(uint32_t operation)
{
#if SIMULATION
  return CAENComm_Success;
#else
  return ov1720_ChannelConfig(device_handle_, operation);
#endif
}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Read 32-bit register
 *
 * \param   [in]  address  address of the register to read
 * \param   [out] val      value read from register
 * \return  CAENComm Error Code (see CAENComm.h)
 */
CAENComm_ErrorCode v1720CONET2::ReadReg_(DWORD address, DWORD *val)
{
  if (verbosity_ >= 2) {
    std::cout << GetName() << "::ReadReg(" << std::hex << address << ")" << std::endl;
    printf("Module: %d, verbosity: %d\n", this->GetModuleID(), verbosity_);
  }
#if SIMULATION
  return CAENComm_Success;
#else
  return CAENComm_Read32(device_handle_, address, val);
#endif
}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Write to 32-bit register
 *
 * \param   [in]  address  address of the register to write to
 * \param   [in]  val      value to write to the register
 * \return  CAENComm Error Code (see CAENComm.h)
 */
CAENComm_ErrorCode v1720CONET2::WriteReg_(DWORD address, DWORD val)
{

#define SIZE 100

  if(address == 0x8108){
    int nptrs, j;
    void *buffer[SIZE];
    char **strings;

    nptrs = backtrace(buffer, SIZE);
    printf("backtrace() returned %d addresses\n", nptrs);

    strings = backtrace_symbols(buffer, nptrs);

    for (j = 0; j < nptrs; j++)
        printf("%s\n", strings[j]);

    free(strings);
  }

  if (verbosity_ >= 2) std::cout << GetName() << "::WriteReg(" << std::hex << address << "," << val << ")" << std::endl;
#if SIMULATION
  return CAENComm_Success;
#else
  return CAENComm_Write32(device_handle_, address, val);
#endif
}

//
//--------------------------------------------------------------------------------
bool v1720CONET2::ReadReg(DWORD address, DWORD *val)
{
  return (ReadReg_(address, val) == CAENComm_Success);
}

//
//--------------------------------------------------------------------------------
bool v1720CONET2::WriteReg(DWORD address, DWORD val)
{
  return (WriteReg_(address, val) == CAENComm_Success);
}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Poll Event Stored register
 *
 * Check Event Stored register for any event stored
 *
 * \param   [out]  val     Number of events stored
 * \return  CAENComm Error Code (see CAENComm.h)
 */
bool v1720CONET2::Poll(DWORD *val)
{
#if SIMULATION
  return true;
#else
  CAENComm_ErrorCode sCAEN = CAENComm_Read32(device_handle_, V1720_EVENT_STORED, val);
  return (sCAEN == CAENComm_Success);
#endif
}

//! Maximum size of data to read using BLT (32-bit) cycle
#define MAX_BLT_READ_SIZE_BYTES 10000

//
//--------------------------------------------------------------------------------
bool v1720CONET2::CheckEvent()
{
  DWORD vmeStat, eStored;
  this->ReadReg(V1720_VME_STATUS, &vmeStat);
        //-PAA- for debugging 
  if (verbosity_ >= 2){ 
    std::cout << GetName() << "::CheckEvent "<< this->GetModuleID() << " " << vmeStat << std::endl;
    ReadReg_(V1720_EVENT_STORED, &eStored);
    printf("### num events: %d\n", eStored);
  }
  return (vmeStat & 0x1);
}

//
//--------------------------------------------------------------------------------
bool v1720CONET2::ReadEvent(void *wp)
{
  CAENComm_ErrorCode sCAEN;
  //  printf("### ReadEvent (board %d) wp: %p\n", this->GetModuleID(), wp);
  
  /*******************************************************************************************
   **** WARNING: The CAEN doc says that the 4th parameter of CAENComm_BLTRead (BltSize) is
   **** in bytes.  That is FALSE.  It must be in 32-bit dwords.  P-L
   *******************************************************************************************/
  
  DWORD size_remaining_dwords, to_read_dwords, *pdata = (DWORD *)wp;
  int dwords_read_total = 0, dwords_read = 0;
  
  sCAEN = ReadReg_(V1720_EVENT_SIZE, &size_remaining_dwords);
  
  while ((size_remaining_dwords > 0) && (sCAEN == CAENComm_Success)) {
    
    //calculate amount of data to be read in this iteration
    to_read_dwords = (size_remaining_dwords > MAX_BLT_READ_SIZE_BYTES/sizeof(DWORD)) ?
      MAX_BLT_READ_SIZE_BYTES/sizeof(DWORD) : size_remaining_dwords;
    sCAEN = CAENComm_BLTRead(device_handle_, V1720_EVENT_READOUT_BUFFER, (DWORD *)pdata, to_read_dwords, &dwords_read);
    
    if (verbosity_>=2) std::cout << sCAEN << " = BLTRead(handle=" << device_handle_
                                 << ", addr=" << V1720_EVENT_READOUT_BUFFER
                                 << ", pdata=" << pdata
                                 << ", to_read_dwords=" << to_read_dwords
                                 << ", dwords_read returned " << dwords_read << ");" << std::endl;
    
    //increment pointers/counters
    dwords_read_total += dwords_read;
    size_remaining_dwords -= dwords_read;
    pdata += dwords_read;
  }
  
  rb_increment_wp(this->GetRingBufferHandle(), dwords_read_total*sizeof(int));

  // >>> Fill QT bank if ZLE data
  if(this->IsZLEData() && this->config.qt_bank){
    if(!this->ReadQTData((uint32_t*)wp)){
      return false;
    }
  }

  // >>> Fill Smart QT bank if ZLE data.
  // Code for calculating minima lives here too.
  if(this->IsZLEData() && (this->config.smartqt_bank || this->config.minima_bank)){
    if(!this->ReadSmartQTData((uint32_t*)wp)){
      return false;
    }
  }
  if(this->IsZLEData() && this->config.minima_bank){
    if(!this->ReadMinimaData((uint32_t*)wp)){
      return false;
    }
  }
  
  /* increment num_events_in_rb_ AFTER writing the QT bank, or the main
   * thread might read the QT data before we're done writing it */
  this->IncrementNumEventsInRB(); //atomic
  if (sCAEN != CAENComm_Success) 
    cm_msg(MERROR,"ReadEvent", "Communication error: %d", sCAEN);

  return (sCAEN == CAENComm_Success);
}

//
//-------------------------------------------------------------------------------------------
bool v1720CONET2::ReadEventCAEN(void *wp)
{
  CAENComm_ErrorCode sCAEN;

  /*******************************************************************************************
   **** WARNING: The CAEN doc says that the 4th parameter of CAENComm_BLTRead (BltSize) is
   **** in bytes.  That is FALSE.  It must be in 32-bit dwords.  P-L
   *******************************************************************************************/
  
  DWORD to_read_dwords=1000000, *pdata = (DWORD *)wp;
  int dwords_read = 0;
  
  sCAEN = CAENComm_BLTRead(device_handle_
                           , V1720_EVENT_READOUT_BUFFER
                           , (DWORD *)pdata
                           , to_read_dwords
                           , &dwords_read);
  
  if (verbosity_>=2) std::cout << sCAEN << " = BLTRead(handle=" << device_handle_
                               << ", addr=" << V1720_EVENT_READOUT_BUFFER
                               << ", pdata=" << pdata
                               << ", to_read_dwords=" << to_read_dwords
                               << ", dwords_read returned " << dwords_read << ");"<< std::endl;
  
  // if dwords_read == to_read_dwords, 
  // BLT tranfert may have been limited and all the event not transfered...
  if( dwords_read == (int)to_read_dwords) 
    cm_msg(MERROR,"ReadEvent", "BLT transfert of maximum size (%d/%d)"
           , dwords_read, to_read_dwords);
  
  // If no event ready, dwords_read will stay at 0, return
  // Since the event is always smaller than 1000000,
  // BLTRead returns CAENComm_Terminated, not CAENComm_success
  if (dwords_read != 0) {
   
    rb_increment_wp(this->GetRingBufferHandle(), dwords_read*sizeof(int));

    // >>> Fill QT bank if ZLE data
    if(this->IsZLEData() && this->config.qt_bank) {
      if(!this->ReadQTData((uint32_t*)wp)) {
        return false;
      }
    }

    // >>> Fill QT bank if ZLE data. Minima code lives here too (to save
    // reading ZLE data twice).
    if(this->IsZLEData() && (this->config.smartqt_bank || this->config.minima_bank)) {
      if(!this->ReadSmartQTData((uint32_t*)wp)) {
        return false;
      }
    }
    if(this->IsZLEData() && this->config.minima_bank) {
      if(!this->ReadMinimaData((uint32_t*)wp)){
        return false;
      }
    }

    /* increment num_events_in_rb_ AFTER writing the QT bank, or the main
     * thread might read the QT data before we're done writing it */
    this->IncrementNumEventsInRB(); //atomic
  }

  bool result = (sCAEN == CAENComm_Terminated);
  if (result == false) 
          cm_msg(MERROR,"ReadEvent", "V1720 BLT Read return error: %d", sCAEN);
  return result;

}

//
//---------------------------------------------------------------------------------
bool v1720CONET2::ReadQTData(uint32_t *ZLEData){

 

  uint32_t *QTData;             // Pointer to location where to write QT data
  uint32_t* nQTWords;           // Pointer to location where number of QT words is
  void     *wp;                 // RB write pointer

  uint32_t nEnabledChannels;    // Number of channels enabled on the board
  bool     chEnabled[8] = {false};        // Channel enabled flag

  uint32_t iCurrentWord;        // Index of current 32-bit word in the ZLE event
  uint32_t chSize_words;        // Size of the current channel in 32-bit words
  uint32_t words_read;          // Number of words read so far for the current channel
  uint32_t iCurrentSample;      // Index of current sample in the channel
  uint32_t iMinSample=0;          // Index of sample with minimum value
  uint32_t minSampleValue=0xfff;  // Value of min sample
  float    baseline_before;     // baseline before pulse
  float    baseline_after;      // baseline after pulse
  float    baseline_avg;        // baseline avg for integral calulation
  float    baseline_before_prev;// baseline before previous pulse
  float    integral_value;      // value of the integral over the pulse
  uint32_t tmp_value;           // Temp sample value
  bool     goodData;            // Indicates if the data following the control word must be processed or skipped
  bool     prevGoodData;        // goodData of previous word
  uint32_t nStoredSkippedWords; // Number of words to be stored (goodData = true) or skipped (goodData = false)
                                // after the control word
  uint32_t i,j;                 // Loop indices

  // >>> Get write pointer into ring buffer
  int status = rb_get_wp(this->GetRingBufferHandle(), &wp, 100);
//  printf("### ReadQTData (board %d) wp: %p\n", this->GetModuleID(), wp);
//  printf("### ReadQTData (board %d) pZLEData: %p\n", this->GetModuleID(), ZLEData);
  if (status == DB_TIMEOUT) {
    cm_msg(MERROR,"ReadQTData", "Got wp timeout for thread %d (module %d).  Is the ring buffer full?",
        this->GetLink(), this->GetModuleID());
    return false;
  }
  QTData = (uint32_t*)wp;

  // >>> copy some header words
  *QTData++ = *(ZLEData+2); // event counter QTData[0]
  *QTData++ = *(ZLEData+3); // trigger time tag QTData[1]

  // >>> Skip location QTData[2]. Will be used for number of QT and bank version.
  nQTWords = QTData;
  *(nQTWords) = 0;
  QTData++;

  // >>> Figure out channel mapping
  nEnabledChannels = 0;
  uint32_t chMask = ZLEData[1] & 0xFF;
//  printf("### Board %d, ZLEData[0]: 0x%08x\n", this->GetModuleID(), ZLEData[0]);
//  printf("### Board %d, ZLEData[1]: 0x%08x\n", this->GetModuleID(), ZLEData[1]);
//  printf("### Board %d, ZLEData[2]: 0x%08x\n", this->GetModuleID(), ZLEData[2]);
//  printf("### Board %d, ZLEData[3]: 0x%08x\n", this->GetModuleID(), ZLEData[3]);
  for(i=0; i<8; ++i){
    if(chMask & (1<<i)){
      chEnabled[i] = true;
      ++nEnabledChannels;
    }
  }

  /* If all data was ZLE suppressed, the channel mask field will be set to 0. In
   * that case, the QT bank will contain only the header   */
  if(nEnabledChannels==0){
    rb_increment_wp(this->GetRingBufferHandle(), (3 + *nQTWords)*sizeof(uint32_t));
    return true;
  }

  iCurrentWord=4;  //Go to first CH0 size word
  for(i=0; i < 8; i++){

    if(!chEnabled[i]) continue;

    chSize_words = ZLEData[iCurrentWord];  // Read size word
    iCurrentSample = 0;                    // Start processing sample 0
    prevGoodData = true;                   // First word
    baseline_before_prev = 0;              // Default value

    words_read = 1;                        // The size of the "size" word is included in its value
    iCurrentWord++;                        // Go to CH0 control word
    while(words_read < chSize_words){

      /************************ TEMPORARY *************************
       * Check for control word indicator, if not present,
       * print this stuff. This should NOT happen.
       ************************************************************/
      //      assert(ZLEData[iCurrentWord] & 0x40000000);
      if((this->GetModuleID() == 0) && (i == 0)){
        if(!(ZLEData[iCurrentWord] & 0x40000000)){
          printf("### b %u ch %u: Control word has wrong format!\n", this->GetModuleID(), i);
          printf("### b %u ch %u: Control word: 0x%08x\n", this->GetModuleID(), i, ZLEData[iCurrentWord]);
          printf("### b %u ch %u: Before: 0x%08x\n", this->GetModuleID(), i, ZLEData[iCurrentWord-1]);
          printf("### b %u ch %u: After: 0x%08x\n", this->GetModuleID(), i, ZLEData[iCurrentWord+1]);
          printf("### b %u ch %u: chSize_words: %u\n", this->GetModuleID(), i, chSize_words);
          printf("### b %u ch %u: words_read: %u\n", this->GetModuleID(), i, words_read);
          printf("### b %u ch %u: prevGoodData: %d\n", this->GetModuleID(), i, prevGoodData);
        }
      }

      goodData = ((ZLEData[iCurrentWord]>>31) & 0x1);           // 0: skip 1: good
      nStoredSkippedWords = (ZLEData[iCurrentWord] & 0xFFFFF);  // stored/skipped words


      /* "skip" data should always be followed by "good" data unless
       * the end of the event is reached.  Print error in case of
       * two consecutive control word with "skip" field   */
      if(!prevGoodData && !goodData){
        cm_msg(MERROR,"ReadQTData", "Consecutive skip data in module %d", this->GetModuleID());
      }

      if(goodData){
        /* For each time good data is encountered process the next nStoredSkippedWords
         * words.  Get the find the sample minimum value and create a QT bank
         * to store this information.  */

        minSampleValue = 0xfff;
        iMinSample = 0;
        iCurrentWord++; // Go to CH0 data word 0
        words_read++;

        //Get baseline before pulse.  Use first 6 samples
        tmp_value = 0;
        for(j=0; j < 3; ++j){
          tmp_value += (ZLEData[iCurrentWord + j] & 0xFFF); //First sample in word
          tmp_value += ((ZLEData[iCurrentWord + j] >> 16) & 0xFFF); //Second sample in word
        }
        baseline_before = tmp_value/6.;  //avg

        // If there are consecutive ZLE blocks (the previous word was also good)
        // we are probably starting in the middle of the second pulse (due to the
        // postsamples of the previous block). Set the baseline_before of this
        // block to be the same as that of the previous block.
        // prevGoodData is initialized to true, so make sure we don't use this
        // logic for the first pulse of an event.
        if (prevGoodData && words_read > 1) {
          baseline_before = baseline_before_prev;
        }
        baseline_before_prev = baseline_before;

        //Get baseline after pulse.  Use last 6 samples
        tmp_value = 0;
        for(j = nStoredSkippedWords-3; j < nStoredSkippedWords; ++j){
          tmp_value += (ZLEData[iCurrentWord + j] & 0xFFF); //First sample in word
          tmp_value += ((ZLEData[iCurrentWord + j] >> 16) & 0xFFF); //Second sample in word
        }
        baseline_after = tmp_value/6.;  //avg

        //Get baseline avg for integral calculation
        baseline_avg = (baseline_before);

        //Calculate integral
        integral_value = 0;
        for(j=0; j < nStoredSkippedWords; j++){
          integral_value += baseline_avg - (ZLEData[iCurrentWord] & 0xFFF);
          integral_value += baseline_avg - ((ZLEData[iCurrentWord] >> 16) & 0xFFF);

                                        // Pulse amplitude
          tmp_value = (ZLEData[iCurrentWord] & 0xFFF);
          if(tmp_value < minSampleValue){
            iMinSample = iCurrentSample + j*2;
            minSampleValue = tmp_value;
          }
          tmp_value = ((ZLEData[iCurrentWord] >> 16) & 0xFFF);
          if(minSampleValue > tmp_value){
            iMinSample = iCurrentSample + j*2 + 1;
            minSampleValue = tmp_value;
          }

          // Go to next data word.  If all data words have been process, this
          // skips to the next control word, which should be "bad" data
          iCurrentWord++;
          words_read++;
        }

        // Don't allow integral to be negative. If we did, we'd get a huge value
        // when casting to uint32_t.
        if (integral_value < 0) {
          integral_value = 0;
        }

        /* Package QT data in three 32-bit words:
         * word 0: 0xA0BBBCCC
         *  A : Channel number
         *  B : Baseline before pulse
         *  C : Baseline after pulse
         * word 1: 0xDDDDEEEE
         *  D : Num first sample in ZLE data
         *  E : Num last sample in ZLE data
         * word 2: 0x00FFFFFF
         *  F : Integral value
         * word 3: 0xGGGGHHHH
         *  G : iMinSample value
         *  H : minSampleValue value
         *
         * notes: The maximum value of the integral occurs when the pulse is maximum (V = 2^12 = 4096)
         *        over the whole time interval (2500 samples) => 4096*2500 = 10 240 000, which requires
         *        24 bits.  The baselines each require 12 bits.  */
        *QTData = ((i << 28) & 0xF0000000) | ((((uint32_t)baseline_before) << 12) & 0x00FFF000) | (((uint32_t)baseline_after) & 0x00000FFF);
        QTData++;
        //2 samples per word
        *QTData = ((iCurrentSample << 16) & 0xFFFF0000) | ((iCurrentSample + nStoredSkippedWords*2) & 0x0000FFFF);
        QTData++;
        *QTData = (uint32_t)integral_value;
        QTData++;
        *QTData = ((iMinSample & 0xFFFF) << 16) | (minSampleValue & 0xFFFF);
        QTData++;

        (*nQTWords) += 4; // increment number of QT words

//        if(i==0){
//          printf("### Wrote QT word 0. Board: %u, Chan: %u, baseline_before: %u, baseline_after: %u\n",
//              this->GetModuleID(), i, baseline_before, baseline_after);
//          printf("### Full word: 0x%08x\n", *(QTData-2));
//          printf("### Wrote QT word 1. Board: %u, Chan: %u, integral_value: %u\n",
//              this->GetModuleID(), i, integral_value);
//          printf("### Full word: 0x%08x\n", *(QTData-1));
//        }
      }
      else{
        /* Data is bad, skip the next nStoredSkippedWords words */

        iCurrentWord++; //Go to next control word, which should be "good" data
        words_read++;
      }

      prevGoodData=goodData;
      iCurrentSample += (nStoredSkippedWords*2); //2 samples per 32-bit word

//      if((i==0)&&(this->GetModuleID()==0)){
//        printf("### iCurrentSample: %u\n", iCurrentSample);
//        printf("### iCurrentSample + nStoredSkippedWords*2: %u\n", iCurrentSample + nStoredSkippedWords*2);
//        printf("### nQTWords: %u\n", (*nQTWords));
//      }
    }
  }

  rb_increment_wp(this->GetRingBufferHandle(), (3 + *nQTWords)*sizeof(uint32_t));
  //if (this->GetModuleID() == 0) printf("### ReadQTData: nQTWords: %u\n", *nQTWords);


  return true;
}
#if 0
//        for(j=0; j < nStoredSkippedWords; j++){
//
//          tmp_value = (ZLEData[iCurrentWord] & 0xFFF);
//          if(tmp_value < minSampleValue){
//            iMinSample = iCurrentSample + j*2;
//            minSampleValue = tmp_value;
//          }
//          tmp_value = ((ZLEData[iCurrentWord] >> 16) & 0xFFF);
//          if(minSampleValue > tmp_value){
//            iMinSample = iCurrentSample + j*2 + 1;
//            minSampleValue = tmp_value;
//          }
//
//          // Go to next data word
//          iCurrentWord++;
//          words_read++;
//        }

//        // Make minSampleValue the amplitude (positive) between baseline and minimum
//        minSampleValue = baseline-minSampleValue;

//        /* Package QT data in one 32-bit word like (0xABBBCCCC) where:
//         * A : Channel number
//         * B : Index of minimum sample
//         * C : Value of minimum sample
//         *
//         * note: Is ANDing necessary? Might gain performance removing it */
//        *QTData = ((i << 28) & 0xF0000000) | ((iMinSample << 16) & 0x0FFF0000) | (minSampleValue & 0x0000FFFF);

//        if(i==0){
//          printf("### Wrote QT word. Board: %u, Chan: %u, iMinSample: %u, minSampleValue: %u\n", this->GetModuleID(), i, iMinSample, minSampleValue);
//          printf("### Full word: 0x%08x\n", *(QTData-1));
//        }
#endif


bool v1720CONET2::ReadSmartQTData(uint32_t *ZLEData){
  // This function fills the smart QT information into the ring buffer,
  // and also calculates information about the minima of each channel,
  // which is stored in memory until a later function writes it the buffer.
  // This approach simplifies the logic in this function, without requiring
  // us to parse the ZLE data twice.


#if POSTPONE_CHARGE_FILTERING
  uint32_t QTDataArr[MAX_QT_WORDS];
  uint32_t *QTData = QTDataArr;
#else
  uint32_t *QTData;             // Pointer to location where to write QT data
#endif
  uint32_t* nQTWords;           // Pointer to location where number of QT words is
  void     *wp;                 // RB write pointer

  uint32_t nEnabledChannels;    // Number of channels enabled on the board
  bool     chEnabled[8] = {false};        // Channel enabled flag

  uint32_t iCurrentWord;        // Index of current 32-bit word in the ZLE event
  uint32_t chSize_words;        // Size of the current channel in 32-bit words
  uint32_t words_read;          // Number of words read so far for the current channel
  uint32_t iCurrentSample;      // Index of current sample in the channel
  bool     goodData;            // Indicates if the data following the control word must be processed or skipped
  bool     prevGoodData;        // goodData of previous word
  uint32_t nStoredSkippedWords; // Number of words to be stored (goodData = true) or skipped (goodData = false)
                                // after the control word
  uint32_t i,j;                 // Loop indices
  bool     previousBlockHadPulse; // Have a smart QT pulse in previous block?

  for (int i = 0; i < 8; i++) {
    channel_minima_[this->GetModuleID()][i] = 4096;
  }

  if (this->config.smartqt_bank) {
    // >>> Get write pointer into ring buffer
    int status = rb_get_wp(this->GetRingBufferHandle(), &wp, 100);
    if (status == DB_TIMEOUT) {
      cm_msg(MERROR,"ReadSmartQTData", "Got wp timeout for thread %d (module %d).  Is the ring buffer full?",
          this->GetLink(), this->GetModuleID());
      return false;
    }

#if POSTPONE_CHARGE_FILTERING
#else
    QTData = (uint32_t*)wp;
#endif

    // >>> copy some header words
    *QTData++ = *(ZLEData+2); // event counter QTData[0]
    *QTData++ = *(ZLEData+3); // trigger time tag QTData[1]

    // >>> Skip location QTData[2]. Will be used for number of QT and bank version.
    nQTWords = QTData;
    *(nQTWords) = 0;
    QTData++;
  }

  // >>> Figure out channel mapping
  nEnabledChannels = 0;
  uint32_t chMask = ZLEData[1] & 0xFF;
  for(i=0; i<8; ++i){
    if(chMask & (1<<i)){
      chEnabled[i] = true;
      ++nEnabledChannels;
    }
  }

  /* If all data was ZLE suppressed, the channel mask field will be set to 0. In
   * that case, the QT bank will contain only the header   */
  if(nEnabledChannels==0) {
    if (this->config.smartqt_bank) {
#if POSTPONE_CHARGE_FILTERING
      uint32_t* QTData_final = (uint32_t*)wp;
      *QTData_final++ = QTData[0]; // event counter
      *QTData_final++ = QTData[1]; // trigger time tag
      *QTData_final++ = 0; // nQTWords
#endif

      rb_increment_wp(this->GetRingBufferHandle(), (3 + *nQTWords)*sizeof(uint32_t));
    }
    return true;
  }

  if (this->config.smartqt_bank) {
    for (int i = 0; i < 8; i++) {
      if (pmts[i]) {
        delete pmts[i];
        pmts[i] = NULL;
      }
    }
  }

  iCurrentWord=4;  //Go to first CH0 size word
  for(i=0; i < 8; i++){

    if(!chEnabled[i]) continue;

    if (this->config.smartqt_bank) {
      pmts[i] = new PMT();
      pmt = pmts[i];
    }

    chSize_words = ZLEData[iCurrentWord];  // Read size word
    iCurrentSample = 0;                    // Start processing sample 0
    prevGoodData = true;                   // First word

    words_read = 1;                        // The size of the "size" word is included in its value
    iCurrentWord++;                        // Go to CH0 control word
    previousBlockHadPulse = false;
    bool firstBlock = true;

    ResetForNewChannel();

    while(words_read < chSize_words){

      /************************ TEMPORARY *************************
       * Check for control word indicator, if not present,
       * print this stuff. This should NOT happen.
       ************************************************************/
      //      assert(ZLEData[iCurrentWord] & 0x40000000);
      if((this->GetModuleID() == 0) && (i == 0)){
        if(!(ZLEData[iCurrentWord] & 0x40000000)){
          printf("### b %u ch %u: Control word has wrong format!\n", this->GetModuleID(), i);
          printf("### b %u ch %u: Control word: 0x%08x\n", this->GetModuleID(), i, ZLEData[iCurrentWord]);
          printf("### b %u ch %u: Before: 0x%08x\n", this->GetModuleID(), i, ZLEData[iCurrentWord-1]);
          printf("### b %u ch %u: After: 0x%08x\n", this->GetModuleID(), i, ZLEData[iCurrentWord+1]);
          printf("### b %u ch %u: chSize_words: %u\n", this->GetModuleID(), i, chSize_words);
          printf("### b %u ch %u: words_read: %u\n", this->GetModuleID(), i, words_read);
          printf("### b %u ch %u: prevGoodData: %d\n", this->GetModuleID(), i, prevGoodData);
        }
      }

      goodData = ((ZLEData[iCurrentWord]>>31) & 0x1);           // 0: skip 1: good
      nStoredSkippedWords = (ZLEData[iCurrentWord] & 0xFFFFF);  // stored/skipped words


      /* "skip" data should always be followed by "good" data unless
       * the end of the event is reached.  Print error in case of
       * two consecutive control word with "skip" field   */
      if(!prevGoodData && !goodData){
        cm_msg(MERROR,"ReadQTData", "Consecutive skip data in module %d", this->GetModuleID());
      }

      if(goodData){
        /* For each time good data is encountered process the next nStoredSkippedWords
         * words.  Get the find the sample minimum value and create a QT bank
         * to store this information.  */

        iCurrentWord++; // Go to CH0 data word 0
        words_read++;

        samples.clear();
        samples.resize(nStoredSkippedWords * 2);

        if (prevGoodData) {
          // If there are consecutive ZLE blocks (the previous word was also good)
          // we are probably starting in the middle of the second pulse (due to the
          // postsamples of the previous block).
          // prevGoodData is initialized to true, so make sure we don't use this
          // logic for the first pulse of an event.
          if (previousBlockHadPulse && this->config.smartqt_bank) {
            pmt->GetLastPulse()->SetConfidence(DS::QT::ADJACENT_BLOCK);
          }

          // And now set a flag so we'll set the confidence of the pulses in this block
          // to 255 too
          IsAdjacent = true;
        } else {
          IsAdjacent = false;
        }


        for(j=0; j < nStoredSkippedWords; j++){
          samples[j*2] = (ZLEData[iCurrentWord] & 0xFFF);
          samples[j*2 + 1] = ((ZLEData[iCurrentWord] >> 16) & 0xFFF);
          iCurrentWord++;
          words_read++;

          // Keep a record of the minimum value this channel reaches.
          channel_minima_[this->GetModuleID()][i] = std::min(channel_minima_[this->GetModuleID()][i], (int)samples[j*2]);
          channel_minima_[this->GetModuleID()][i] = std::min(channel_minima_[this->GetModuleID()][i], (int)samples[j*2 + 1]);
        }

        if (this->config.smartqt_bank) {

          pulseState.Offset = iCurrentSample;
          pulseState.IsLowGain = false;
          pulseState.IsV1740 = false;

          int oldCount = pmt->GetPulseCount();

          FindPulses(firstBlock,
                     pfConfig.ClosePulseLookAhead,
                     pfConfig.ClosePulseNearEndOfBlock,
                     pfConfig.ExtendPulseEnd,
                     pfConfig.EnableSubPeaks,
                     pfConfig.EnablePulseSplitting);

          previousBlockHadPulse = (pmt->GetPulseCount() != oldCount);

        } // End of smart QT

        firstBlock = false;
      } else {
        /* Data is bad, skip the next nStoredSkippedWords words */

        iCurrentWord++; //Go to next control word, which should be "good" data
        words_read++;
      }

      prevGoodData=goodData;
      iCurrentSample += (nStoredSkippedWords*2); //2 samples per 32-bit word

    }

    if (this->config.smartqt_bank) {
      // End of channel
      UpdateBaselinesAndCutOnCharge(pfConfig.ChargeCut);
    }
  }


  if (this->config.smartqt_bank) {
    ConvertPMTsToBanks(QTData, nQTWords);
    // And actually write to the ring bugger.
    rb_increment_wp(this->GetRingBufferHandle(), (3 + *nQTWords)*sizeof(uint32_t));
  }

  //if (this->GetModuleID() == 0) printf("### ReadSmartQTData: nQTWords: %u\n", *nQTWords);

  return true;
}

bool v1720CONET2::ReadMinimaData(uint32_t *ZLEData){
  uint32_t *minData;             // Pointer to location where to write QT data
  void     *wp;                 // RB write pointer

  // >>> Get write pointer into ring buffer
  int status = rb_get_wp(this->GetRingBufferHandle(), &wp, 100);
  if (status == DB_TIMEOUT) {
    cm_msg(MERROR,"ReadMinimaData", "Got wp timeout for thread %d (module %d).  Is the ring buffer full?",
        this->GetLink(), this->GetModuleID());
    return false;
  }
  minData = (uint32_t*)wp;

  // First some header words
  *minData++ = *(ZLEData+2); // event counter QTData[0]
  *minData++ = *(ZLEData+3); // trigger time tag QTData[1]

  // Now the data - 8 channels packed into 4 32-bit words
  for (int i = 0; i < 8; i += 2) {
    *minData++ = (((channel_minima_[this->GetModuleID()][i] << 16) & 0xFFFF0000) | (channel_minima_[this->GetModuleID()][i + 1] & 0x0000FFFF));
  }

  rb_increment_wp(this->GetRingBufferHandle(), 6*sizeof(uint32_t));

  return true;
}

//
//--------------------------------------------------------------------------------
bool v1720CONET2::FetchHeaderNextEvent(uint32_t * header)
{
  DWORD *src;

  int status = rb_get_rp(this->GetRingBufferHandle(), (void**)&src, 100);
  if (status == DB_TIMEOUT) {
    cm_msg(MERROR,"FetchHeaderNextEvent", "Got rp timeout for module %d", this->GetModuleID());
    return false;
  }

  memcpy(header, src, 32);

  return true;
}

//
//--------------------------------------------------------------------------------
bool v1720CONET2::DeleteNextEvent()
{
  int status;
  DWORD *rp;

  status = rb_get_rp(this->GetRingBufferHandle(), (void**)&rp, 100);
  if (status == DB_TIMEOUT) {
    cm_msg(MERROR,"DeleteNextEvent", "Got rp timeout for module %d", this->GetModuleID());
    return false;
  }

  uint32_t size_words = *rp & 0x0FFFFFFF;

  this->DecrementNumEventsInRB(); //atomic
  rb_increment_rp(this->GetRingBufferHandle(), size_words*4);


  return true;
}

//
//--------------------------------------------------------------------------------
bool v1720CONET2::FillEventBank(char * pevent)
{
  if (! this->IsConnected()) {
    cm_msg(MERROR,"FillEventBank","Board %d disconnected", this->GetModuleID());
    return false;
  }

  DWORD *src=NULL;
  DWORD *dest=NULL;

  int status = rb_get_rp(this->GetRingBufferHandle(), (void**)&src, 500);
  if (status == DB_TIMEOUT) {
    cm_msg(MERROR,"FillEventBank", "Got rp timeout for module %d", this->GetModuleID());
    printf("### num events: %d\n", this->GetNumEventsInRB());
    return false;
  }

#ifdef DEBUGTHREAD
  printf("Event size (words): %u\n", *src & 0x0FFFFFFF);
  printf("Event Counter: %u\n", *(src+2) & 0x00FFFFFF);
  printf("TTT: %u\n", *(src + 3));
  int nbWords = *src & 0x0FFFFFFF;
  for (int i=0; i< nbWords && i<1000; ++i) 
  {
    if (i%8 == 0) printf("\n%d: ",i);
    printf ("%08x ", src[i]);
  }
  assert((*src & 0xF0000000) == 0xA0000000);  //Event header indicator
#endif

  if ((*src & 0xF0000000) != 0xA0000000){
    cm_msg(MERROR,"FillEventBank","Incorrect hearder for board:%d (0x%x)", this->GetModuleID(), *src);
    return false;
  }

  uint32_t size_words = *src & 0x0FFFFFFF;
  uint32_t size_copied = size_words;

  // >>> create data bank
  char bankName[5];
  snprintf(bankName, sizeof(bankName), "W2%02d", this->GetModuleID());
  if(this->IsZLEData()){
    snprintf(bankName, sizeof(bankName), "ZL%02d", this->GetModuleID());
  }
  else{
    snprintf(bankName, sizeof(bankName), "W2%02d", this->GetModuleID());
  }
 // printf("Bank size (before %s): %u, event size: %u\n", bankName, bk_size(pevent), size_words);
  bk_create(pevent, bankName, TID_DWORD, (void **)&dest);


  uint32_t limit_size = (32*222800-bk_size(pevent))/4; // what space is left in the event (in DWORDS)  
  if (size_words > limit_size) {
//    printf("Event with size: %u (Module %02d) bigger than max %u, event truncated\n", size_words, this->GetModuleID(), limit_size);
    cm_msg(MERROR,"FillEventBank","Event with size: %u (Module %02d) bigger than max %u, event truncated", size_words, this->GetModuleID(), limit_size);
    if(this->IsZLEData()){
      uint32_t toBeCopyed = 4; // Starting with the header
	  // We need to find out how many channels we can copy before reaching the limit...
	  int i;
	  for (i=8; i>0 ; --i){ //We have potentially 8 channels to copy
        uint32_t channelSize = 0;
	    channelSize = *(src+toBeCopyed); // Get the size of the data for this channel
	    if (toBeCopyed + channelSize > limit_size) break; 
	    toBeCopyed += channelSize; // We have enough space for this channel
	  }
	  size_copied = toBeCopyed + i; //This it the size of the headers, the filled channel, and the "empty channels size" padding.
//    printf("will be copied: %u out of %u (%d channels)\n", size_copied, size_words, (8-i));
      cm_msg(MERROR,"FillEventBank","will be copied: %u out of %u (%d channels)", size_copied, size_words, (8-i));
      *(src + 0) = 0xA0000000 + size_copied; // Adjust the event size
	  for ( ; i>0 ; --i){
	    *(src + toBeCopyed+(i-1)) = (uint32_t) 0x1; // Pad the empty channel size = 1 DWORDS
	  }
	}
	else {
//      printf("Raw mode with long waveforms, exceeding the limit: event skipped\n");
      cm_msg(MERROR,"FillEventBank","Raw mode with long waveforms, exceeding the limit: event skipped");
      *(src + 0) = 0xA0000004; // Event Size set to 0 data (4 DWORDS for the the header ==> TO be checked !)
	  size_copied = 4;
	}
  } 
  
  memcpy(dest, src, size_copied*sizeof(uint32_t));

  this->DecrementNumEventsInRB(); //atomic
  rb_increment_rp(this->GetRingBufferHandle(), size_words*sizeof(uint32_t));

  //Close data bank
  bk_close(pevent, dest + size_copied);

 // printf("Bank size (after %s): %u\n" , bankName, bk_size(pevent));
  // >>> Fill QT bank if ZLE data
  if(this->IsZLEData() && this->config.qt_bank){
    this->FillQTBank(pevent, dest);
  }

  // >>> Fill smart QT bank if ZLE data
  if(this->IsZLEData() && this->config.smartqt_bank) {
    this->FillSmartQTBank(pevent, dest);
  }
  if(this->IsZLEData() && this->config.minima_bank) {
    this->FillMinimaBank(pevent, dest);
  }
  return true;

}

/// This function should be IDENTICAL to that used in RAT.
void v1720CONET2::ResetForNewChannel()
{
  CurrentDefaultBaseline = DefaultBaseline;
  channelState.BaselineInt = 0;
  channelState.BaselineSquareInt = 0;
  channelState.BaselineSamples = 0;
  channelState.IsBlockCombinedWithPrevious.clear();
  channelState.Saturated = false;
}

/// This function should be IDENTICAL to that used in RAT.
void v1720CONET2::ResetForNewPulse()
{
  pulseState.Charge = 0;
  pulseState.ChargePeak = 0;
  pulseState.ChargeConv = 0;
  pulseState.Start = 0;
  pulseState.Stop = 0;
  pulseState.Conf = 0;
  pulseState.MaxV = 5000;
  pulseState.MaxD = 0;
  pulseState.MaxVT.clear();
  pulseState.MaxVindex = 0;
  pulseState.MaxDindex = 0;
  pulseState.Charge_MaxD = 0;
  pulseState.Charge_MaxV = 0;
  pulseState.Charge_Frac = 0;
  pulseState.GlobalMaxD = 0;
  pulseState.BaseV = 0;
  pulseState.BaselineInt = 0;
  pulseState.BaselineSquareInt = 0;
  pulseState.BaselineSamples = 0;
  pulseState.MinVT = 0;
  pulseState.MinV = 0;
  pulseState.GlobalMaxV = 5000;
  pulseState.GlobalMaxVT = 0;
  pulseState.CanSplit = false;
  pulseState.IsSplit = false;
}

/// This function should be IDENTICAL to that used in RAT.
void v1720CONET2::FindPulses(bool firstBlock, int closePulseLookAhead, int closePulseNearEndOfBlock, bool extendPulseEnd, bool enableSubPeaks, bool enablePulseSplitting)
{
  int sampsize = samples.size();
  CalculateDerivativesFromSamples();

  Bool_t InPulse = false;
  UShort_t PeakVoltage = 0;
  UShort_t GlobalPeakVoltage = 5000;

  ResetForNewPulse();

  for (int i = 1; i < sampsize - 2; i++) {
    if (samples[i] == 0) {
      saturationFail = true;
      channelState.Saturated = true;
    }

    if (InPulse) {
      pulseState.Charge += 4096 - samples[i];
      if (samples[i] < pulseState.MaxV) {
        if (!enableSubPeaks) {
          pulseState.MaxVT[0] = i;
          pulseState.ChargePeak = pulseState.Charge;
        }
        pulseState.MaxV = samples[i];
      }
      if (enablePulseSplitting &&
          TMath::Abs((Float_t)samples[i]-pulseState.BaseV) < 3*pfConfig.StopVolt &&
          AbsDeriv[i] < pfConfig.StopDeriv &&
          pulseState.Charge > pfConfig.SplitPulseCharge &&
          pulseState.MaxVT.size()) {
        pulseState.CanSplit = true;
      }

      if (pulseState.MinV < samples[i]) {
        pulseState.MinVT = i;
        pulseState.MinV = samples[i];
      }
      if (pulseState.GlobalMaxV > samples[i]) {
        pulseState.GlobalMaxV = samples[i];
        pulseState.GlobalMaxVT = i;
      }
      if (AbsDeriv[i] > TMath::Abs(pulseState.MaxD)) {
        pulseState.MaxD = Deriv[i];
        if (TMath::Abs(pulseState.MaxD) > TMath::Abs(pulseState.GlobalMaxD)) {
          pulseState.GlobalMaxD = pulseState.MaxD;
        }
      }
      if (enableSubPeaks) {
        if (samples[i] == 0 && samples[i-1] > 0) {
          pulseState.MaxVT.push_back(i);
          PeakVoltage = samples[i];
          if (PeakVoltage < GlobalPeakVoltage) {
            pulseState.ChargePeak = pulseState.Charge;
            GlobalPeakVoltage = PeakVoltage;
            pulseState.MaxVindex = i;
          }
        } else if ((Deriv[i-1] * Deriv[i] <= 0) && (Deriv[i-1] * Deriv[i+1] <= 0) && Deriv[i-1] != 0) {
          if (Deriv[i]<=0 && Deriv[i+1]<=0) {
            pulseState.MaxD = Deriv[i];
          } else if (Deriv[i]>=0 && Deriv[i+1]>=0) {
            if (pulseState.CanSplit &&
                pulseState.MaxD < pfConfig.SubPeakDerivThreshold) {
              UInt_t NewPulseCharge = 0;
              for (int j = i; j >= pulseState.MinVT; --j) {
                NewPulseCharge += 4096 - samples[j];
              }
              pulseState.Charge -= NewPulseCharge;
              UShort_t chanBaselineInt = channelState.BaselineInt;
              UShort_t chanBaselineSamples = channelState.BaselineSamples;
              UShort_t chanBaselineSquareInt = channelState.BaselineSquareInt;
              UShort_t BaselineSamples = pulseState.BaselineSamples;
              UShort_t BaselineInt = pulseState.BaselineInt;
              UShort_t BaselineSquareInt = pulseState.BaselineSquareInt;
              UShort_t Start = pulseState.MinVT;
              Short_t MaxD = pulseState.MaxD;
              int end = pulseState.MinVT-1;
              std::vector<UShort_t> newMaxVT;
              for (UShort_t j = 0; j < pulseState.MaxVT.size(); ++j) {
                if (pulseState.MaxVT[j] > end) {
                  newMaxVT.push_back(pulseState.MaxVT[j]);
                  pulseState.MaxVT.erase(pulseState.MaxVT.begin() + j);
                  --j;
                }
              }
              ClosePulseAndSetVariables(end, firstBlock, false,closePulseLookAhead);
              pulseState.MaxVT = newMaxVT;
              pulseState.Charge = NewPulseCharge;
              pulseState.Start = Start;
              GlobalPeakVoltage = samples[i];
              pulseState.MaxD = Deriv[i];
              pulseState.GlobalMaxD = Deriv[i];
              channelState.BaselineInt = chanBaselineInt;
              channelState.BaselineSamples = chanBaselineSamples;
              channelState.BaselineSquareInt = chanBaselineSquareInt;
              pulseState.BaselineSamples = BaselineSamples;
              pulseState.BaselineInt = BaselineInt;
              pulseState.BaselineSquareInt = BaselineSquareInt;
              pulseState.MaxD = MaxD;
              pulseState.IsSplit = true;
            }
            PeakVoltage = 5000;
            int peaktime = 0;
            UShort_t ChargePeak = pulseState.Charge + samples[i] + samples[i-1] + samples[i-2] - 12288;
            for (int j = i-2; j <= i+2; ++j) {
              ChargePeak += 4096 - samples[j];
              if (samples[j] < PeakVoltage) {
                peaktime = j;
                PeakVoltage = samples[j];
                if (PeakVoltage < GlobalPeakVoltage) {
                  pulseState.ChargePeak = ChargePeak;
                  GlobalPeakVoltage = PeakVoltage;
                  pulseState.MaxVindex = j;
                }
              }
            }
            if (PeakVoltage - pulseState.MinV < pfConfig.SubPeakDerivThreshold &&
                !(pulseState.MaxVT.size() && peaktime == pulseState.MaxVT[pulseState.MaxVT.size()-1])) {
              pulseState.MaxVT.push_back(peaktime);
              pulseState.MinV = 0;
              pulseState.MinVT = i;
              pulseState.MaxD = Deriv[i];
              pulseState.CanSplit = false;
            }
          }
        }
      }

      if ((TMath::Abs((Float_t)samples[i]-pulseState.BaseV) < pfConfig.StopVolt && AbsDeriv[i] < pfConfig.StopDeriv && samples[i] > 0)
          || (i == (sampsize - closePulseNearEndOfBlock))) {
        if (i < (sampsize - 2*closePulseLookAhead)) {

          bool closepulsebool = true;
          for (int j=closePulseLookAhead+1+i; j<=(2*closePulseLookAhead+i); ++j)
            if (TMath::Abs((Float_t)samples[j]-pulseState.BaseV) > pfConfig.ClosePulseHeightThresh || AbsDeriv[j] > pfConfig.ClosePulseDerivThresh)
              closepulsebool = false;

          if (closepulsebool) {
            pulseState.CanSplit = false;
            if (!pulseState.MaxVT.size()) pulseState.MaxVT.push_back(pulseState.GlobalMaxVT);
            ClosePulseAndSetVariables(i, firstBlock, extendPulseEnd,closePulseLookAhead);
            InPulse = kFALSE;

            if (enableSubPeaks) {
              GlobalPeakVoltage = 5000;
              pulseState.MaxD = Deriv[i];
              pulseState.GlobalMaxD = Deriv[i];
            }
          }
        } else if (i == (sampsize - closePulseNearEndOfBlock)) {
          if (!pulseState.MaxVT.size()) pulseState.MaxVT.push_back(pulseState.GlobalMaxVT);
          ClosePulseAndSetVariables(i, firstBlock, extendPulseEnd,closePulseLookAhead);
          InPulse = kFALSE;
        }
      }
    } else {
      if (AbsDeriv[i] > pfConfig.DerivThreshold || samples[i] < DefaultBaseline - 50.) {
        // Starting a new pulse
        //look back.
        pulseState.Start = TMath::Max(i-closePulseLookAhead,0);
        pulseState.MaxVT.clear();
        pulseState.MinVT = pulseState.Start;
        pulseState.MinV = samples[pulseState.Start];
        if (!enableSubPeaks) {
          pulseState.MaxVT.push_back(i);
          pulseState.MaxV = samples[i];
        }
        pulseState.Charge = 4096 - samples[i];
        for (int j = pulseState.Start; j < i; ++j) {
          pulseState.Charge += 4096 - samples[j];
          if (pulseState.BaselineSamples) {
            --pulseState.BaselineSamples;
            pulseState.BaselineSquareInt -= ((UInt_t)samples[j]*samples[j])-(3892*3892);
            pulseState.BaselineInt -= samples[j]-3892;
            --channelState.BaselineSamples;
            channelState.BaselineInt -= samples[j]-3892;
            channelState.BaselineSquareInt -= ((UInt_t)samples[j]*samples[j])-(3892*3892);
          }
        }

        if (pulseState.BaselineSamples > 4) {
          pulseState.BaseV = ((float)pulseState.BaselineInt / (float)pulseState.BaselineSamples) + 3892;
        } else {
          pulseState.BaseV = CurrentDefaultBaseline;
        }
        CurrentDefaultBaseline = pulseState.BaseV;
        PeakVoltage = samples[i];
        GlobalPeakVoltage = 5000;
        InPulse = true;
      } else {
        ++pulseState.BaselineSamples;
        pulseState.BaselineSquareInt += ((UInt_t)samples[i]*samples[i])-(3892*3892);
        pulseState.BaselineInt += samples[i]-3892;

        if (channelState.BaselineSamples < pfConfig.MaxBaselineSamples) {
          ++channelState.BaselineSamples;
          channelState.BaselineInt += samples[i]-3892;
          channelState.BaselineSquareInt += ((UInt_t)samples[i]*samples[i])-(3892*3892);
        }
      }
    }
  }
}

/// This function should be IDENTICAL to that used in RAT.
void v1720CONET2::ClosePulseAndSetVariables(int& i, bool firstBlock, bool extendPulseEnd, int closePulseLookAhead)
{
  if (extendPulseEnd) {
    for (int j = 1; j <= closePulseLookAhead; ++j) {
      pulseState.Charge += 4096 - samples[i+j];
    }
    i += closePulseLookAhead;
  }

  pulseState.Stop = i;

  SetPulseVariables();

  if (channelState.BaselineSamples < 3 && firstBlock) {
    channelState.BaselineInt = 0;
    channelState.BaselineSamples = 0;
    channelState.BaselineSquareInt = 0;
  }

  pulseState.BaselineSamples = 0;
  pulseState.BaselineInt = 0;
  pulseState.BaselineSquareInt = 0;
}

/// This function needs changing compared to that used in RAT.
void v1720CONET2::SetPulseVariables()
{
  if (pulseState.MaxVT.size() && pulseState.MaxVT[pulseState.MaxVT.size()-1] >= pulseState.Stop-1) {
    pulseState.MaxVT.pop_back();
  }

  if (pulseState.MaxVT.size() == 0) {
    if (pulseState.IsSplit) {
      pulse = pmt->GetLastPulse();
      pulseState.Charge += pulse->GetChargeADCRel4096();
      AdjustPulseChargeAndTypeToAvoidOverflow();
      pulse->SetChargeADCRel4096(pulseState.Charge);
      pulse->SetRightEdge(pulseState.Stop + pulseState.Offset);
    }

    ResetForNewPulse();
    return;
  }

  if (pulseState.IsLowGain) {
    pulse = pmt->AddNewLGPulse();
  } else {
    pulse = pmt->AddNewPulse();
  }

  AdjustPulseChargeAndTypeToAvoidOverflow();
  pulse->SetChargeADCRel4096(pulseState.Charge);
  pulse->SetBaselineIntRel3892(pulseState.BaselineInt);
  pulse->SetBaselineSquareIntRel3892(pulseState.BaselineSquareInt);
  pulse->SetBaselineSamples(pulseState.BaselineSamples);
  pulse->SetMinimumADC(pulseState.MaxV);
  /* START OF DIFF
  pulse->SetLeftEdge(pulseState.Start + pulseState.Offset);
  pulse->SetRightEdge(pulseState.Stop + pulseState.Offset);

  for (UInt_t j = 0; j < pulseState.MaxVT.size(); ++j) {
    if (pulseState.MaxVT[j] > 1 && pulseState.MaxVT[j] < samples.size() - 1) {
      pulseState.SubnsSamples.clear();
      for (int i = -2; i < 2; ++i) {
        pulseState.SubnsSamples.push_back(samples[pulseState.MaxVT[j] + i]);
      }
      pulse->SetTimingSamples(pulseState.SubnsSamples);
    }
  }

  for (unsigned int i = 0; i < pulseState.MaxVT.size(); ++i) {
    pulseState.MaxVT[i] += pulseState.Offset;
  }

  pulse->SetSubpeaks(pulseState.MaxVT);
  pulse->SetTimeOffset(channelState.TimeOffset);

  if (pulseState.IsV1740) {
    pulse->SetBoardID(board1740->GetID());
    pulse->SetInputID(rawWF->GetInputID());
    pulse->SetRawBlockStartBin(0);
  } else {
    pulse->SetBoardID(board->GetID());
    pulse->SetInputID(channel->GetInputID());
    pulse->SetRawBlockStartBin(rawBlock->GetStartBin());
  }
  */

  pulse->SetLeftEdge(pulseState.Start);
  pulse->SetRightEdge(pulseState.Stop);
  pulse->SetMaxD(pulseState.MaxD);
  pulse->SetMaxV(pulseState.MaxV);
  pulse->SetMaxVT(pulseState.MaxVT[0]);
  pulse->SetPeakChargeADCRel4096(pulseState.ChargePeak);
  pulse->SetOffset(pulseState.Offset);
  pulse->SetPeakTime(pulseState.MaxVT[0]);
  pulse->SetPrePreSample(samples[pulseState.MaxVT[0] - 2]);
  pulse->SetPreSample(samples[pulseState.MaxVT[0] - 1]);
  pulse->SetPeakVoltage(samples[pulseState.MaxVT[0]]);
  pulse->SetPostSample(samples[pulseState.MaxVT[0] + 1]);
  pulse->SetPulseType(PulseUtil::TYPE_V1720_SMARTQT);
  // This must be last as it relies on other values in pulse.
  pulse->SetConfidence(EvalSQTPulse());
  // END OF DIFF

  ResetForNewPulse();
}

void v1720CONET2::AdjustPulseChargeAndTypeToAvoidOverflow() {
  // THIS IS A UNIQUE IMPLEMENTATION FOR ONLINE
  if (pulseState.Charge > 65535) {
    pulseState.Charge = 65535;
  }
}

void v1720CONET2::UpdateBaselinesAndCutOnCharge(int chargeCutADCRelBaseline, bool lowGain)
{
  /* DIFF
  if (channelState.BaselineSamples <= pfConfig.MinBaselineSamples) {
    // Bad baseline, so set the failure flag
    baselineFail = true;
  }*/

  int pulseCount = lowGain ? pmt->GetLGPulseCount() : pmt->GetPulseCount();

  for (Int_t ipulse = 0; ipulse < pulseCount; ++ipulse) {
    pulse = lowGain ? pmt->GetLGPulse(ipulse) : pmt->GetPulse(ipulse);

// DIFF    if (pulse->GetPulseType() != PulseUtil::TYPE_V1720_SMARTQT) {
      pulse->SetBaselineIntRel3892(channelState.BaselineInt);
      pulse->SetBaselineSquareIntRel3892(channelState.BaselineSquareInt);
      pulse->SetBaselineSamples(channelState.BaselineSamples);
// DIFF    }

    if (pulse->GetChargeADCRelBaseline() < chargeCutADCRelBaseline) {
      pulse = NULL;
      pulseCount--;
/* DIFF      if (lowGain) {
        pmt->RemoveLGPulse(ipulse--);
      } else {*/
        pmt->RemovePulse(ipulse--);
// DIFF      }
      continue;
    }
/* DIFF
    if (pulse->GetPulseType() < PulseUtil::TYPE_V1740 && pmt->GetID() > -1 && pmt->GetID() < 255) {
      motherblock = GetRawBlock();
      pulseState.Offset = motherblock->GetStartBin();
      samples = motherblock->GetSamples();
      FitTime_Block();
      if (pulse->GetPulseType() < PulseUtil::TYPE_V1720_SMARTQT &&
          pulse->GetChargeADCRelBaseline() >= pfConfig.SubPeakChargeCutOff) {
        SetLargePulseSubPeakCharge();
      } else if (pulse->GetPulseType() < PulseUtil::TYPE_V1720_SMARTQT &&
                 (pulse->GetSubpeakCount() > 9 || pulse->GetLeftEdge() == 0 || channelState.Saturated) &&
                 TMath::Abs(pulse->GetChargeResidualFrac()) > pfConfig.ChargeResidualFracLimit &&
                 TMath::Abs(pulse->GetChargeResidualADC()) > pfConfig.ChargeResidualLimit) {
        SetLargePulseSubPeakCharge();
      } else if (pfConfig.DoHiddenPeakFind &&
                 pulse->GetPulseType() < PulseUtil::TYPE_V1720_SMARTQT &&
                 TMath::Abs(pulse->GetChargeResidualFrac()) > pfConfig.ChargeResidualFracLimit &&
                 TMath::Abs(pulse->GetChargeResidualADC()) > pfConfig.ChargeResidualLimit) {
        DS::Pulse* pulseOriginal = pulse;
        pulse = (DS::Pulse*) pulseOriginal->Clone();
        FitTime_BruteForce();
        if (TMath::Abs(pulse->GetChargeResidualADC()) < TMath::Abs(pulseOriginal->GetChargeResidualADC())) {
          pmt->ReplacePulse(ipulse, pulse);
        } else {
          delete pulse;
          pulse = pulseOriginal;
        }
        for (UShort_t j = 0; j < pulse->GetSubpeakCount(); ++j) {
          if (pulse->GetSubpeak(j)-pulseState.Offset > -1 && pulse->GetSubpeak(j)-pulseState.Offset < samples.size()) {
            pulse->AddSubpeakMinimum(samples[pulse->GetSubpeak(j)-pulseState.Offset]);
          } else {
            pulse->AddSubpeakMinimum(0);
          }
        }

      } else {
        for (int j=0; j<pulse->GetSubpeakCount(); ++j) {
          pulse->AddSubpeakMinimum(samples[pulse->GetSubpeak(j)-pulseState.Offset]);
        }
      }

      SetPETimes();
    }

    pulse->CalcTransients();*/
  }
}

// This function should e IDENTICAL to that in rat
void v1720CONET2::CalculateDerivativesFromSamples()
{
  int sampsize = samples.size();

  Deriv.clear();
  AbsDeriv.clear();

  Deriv.push_back(2*((int)samples[1]-(int)samples[0]));
  AbsDeriv.push_back(TMath::Abs(Deriv[0]));

  for (int i = 1; i < sampsize - 2; i++) {
    Deriv.push_back((int)samples[i+1] - (int)samples[i-1]);
    AbsDeriv.push_back(TMath::Abs(Deriv[i]));
  }

  Deriv.push_back(2*((int)samples[sampsize-2]-(int)samples[sampsize-1]));
  AbsDeriv.push_back(TMath::Abs(2*((int)samples[sampsize-2]-(int)samples[sampsize-1])));
}


//
//--------------------------------------------------------------------------------
/**
 * \brief   Fill Qt Bank
 *
 * \param   [in]  pevent  pointer to event buffer
 * \param   [in]  pZLEData  pointer to the data area of the bank
 */
bool v1720CONET2::FillQTBank(char * pevent, uint32_t * pZLEData){

  DWORD *src;
  DWORD *dest;
  uint32_t nQTWords;

  //The read pointer points to the QT data following ZLE Event data
  int status = rb_get_rp(this->GetRingBufferHandle(), (void**)&src, 100);
  if (status == DB_TIMEOUT) {
    cm_msg(MERROR,"FillQTBank", "Got rp timeout for module %d", this->GetModuleID());
    printf("rp timeout, Module %d\n", this->GetModuleID());
    return false;
  }

  char tBankName[5];
  snprintf(tBankName, sizeof(tBankName), "QT%02d",this->GetModuleID());
  bk_create(pevent, tBankName, TID_DWORD, (void **)&dest);

  //Copy QT header
  memcpy(dest, src, 3*sizeof(uint32_t));  //The QT header is 3 words long (see ReadQTData())
  nQTWords = *(src + 2);
  //if (this->GetModuleID() == 0) printf("### FillQTBank: nQTWords: %u\n", nQTWords);

  //Copy QT words
  memcpy(dest+3, src+3, nQTWords*sizeof(uint32_t));

  rb_increment_rp(this->GetRingBufferHandle(), (3 + nQTWords)*sizeof(uint32_t));
  bk_close(pevent, dest + 3 + nQTWords);

  return true;
}

void v1720CONET2::ConvertPMTsToBanks(uint32_t*& QTData, uint32_t*& nQTWords) {
  // Current packing:
  // CBBVVVXX, C: Channel, BB: Bank version, VVV: Peak voltage, XX: pre-pre
  // YYZZCCCC, YY: pre, ZZ: post, CCCC: Charge (4096 - samples)
  // FFFFAAOO, FFFF: Offset (start bin of block), AA: Start bin of pulse in block, OO: Stop bin of pulse in block
  // 0BBBPPSS, BBB: (Baseline sum relative to 3892), PP: Bin of peak of pulse in block, SS: SPE confidence
  // SSSSSSNN, SSSSSS: Sum of baseline squares (rel to 3892), NN: Baseline number of samples
  //
  // Note, if this packing is changed, you MUST also update
  // ebSmartQTFilter::AnalyzeBanks() so it reads the right values.
  //
  // Changelog:
  // v8 - Initial Smart QT implementation
  // v9 - No change to V1720 section, but Event Builder filtering improved.

  for (int p = 0; p < 8; p++) {
    if (!pmts[p]) {
      continue;
    }

    for (int u = 0; u < pmts[p]->GetPulseCount(); u++) {
      pulse = pmts[p]->GetPulse(u);

      uint32_t _Channel = p;
      uint32_t _Version = 9;
      uint32_t _PeakV = pulse->GetPeakVoltage();
      uint32_t _PrePreMV = pulse->GetPrePreSample() - _PeakV;

      *QTData = (((_Channel << 28) & 0xF0000000) | ((_Version << 20) & 0x0FF00000) | ((_PeakV << 8) & 0x000FFF00) | ((_PrePreMV) & 0x000000FF));
      QTData++;

      uint32_t _PreMV = pulse->GetPreSample() - _PeakV;
      uint32_t _PostMV = pulse->GetPostSample() - _PeakV;
      uint32_t _Charge = pulse->GetChargeADCRel4096();

      *QTData = (((_PreMV << 24) & 0xFF000000) | ((_PostMV << 16) & 0x00FF0000) | (_Charge & 0x0000FFFF));
      QTData++;

      uint32_t _Offset = pulse->GetOffset();
      uint32_t _Start = pulse->GetLeftEdge();
      uint32_t _Stop = pulse->GetRightEdge();

      *QTData = (((_Offset << 16) & 0xFFFF0000) | ((_Start << 8) & 0x0000FF00) | (_Stop & 0x000000FF));
      QTData++;

      uint32_t _BaselineB = pulse->GetBaselineIntRel3892();
      uint32_t _PeakT = pulse->GetPeakTime();
      uint32_t _Conf = pulse->GetConfidence();

      *QTData = (((_BaselineB << 16) & 0x0FFF0000) | ((_PeakT << 8 ) & 0x0000FF00) | (_Conf & 0x000000FF));
      QTData++;

      uint32_t _BaselineS = pulse->GetBaselineSquareIntRel3892();
      uint32_t _BaselineN = pulse->GetBaselineSamples();

      *QTData = (((_BaselineS << 8) & 0xFFFFFF00) | (_BaselineN & 0x000000FF));
      QTData++;

      (*nQTWords) += NUM_SQ_WORDS; // increment number of QT words
    }
  }
}

bool v1720CONET2::LoadSmartQTConfig() {
  std::ifstream myfile;
  myfile.open("/home/deap/pro/FrontEnd/v1720mt/SmartQTArrays.txt");

  double dummy;
  myfile >> dummy;
  myfile >> spePdf.MaxVbins;
  myfile >> spePdf.ChargeFracbins;
  myfile >> dummy;
  myfile >> dummy;
  myfile >> dummy;
  myfile >> spePdf.MaxVmin;
  myfile >> spePdf.MaxVmax;
  myfile >> spePdf.ChargeFracmin;
  myfile >> spePdf.ChargeFracmax;
  for (int z = 0; z < 22; ++z) {
    for (int x = 0; x < spePdf.MaxVbins; ++x) {
      for (int y = 0; y < spePdf.ChargeFracbins; ++y) {
        myfile >> dummy;
        spePdf.MaxV_ChargeFrac.push_back(dummy);
      }
    }
  }
  myfile >> dummy;
  myfile >> dummy;
  myfile >> spePdf.MaxDbins;
  myfile >> dummy;
  myfile >> dummy;
  myfile >> dummy;
  myfile >> dummy;
  myfile >> dummy;
  myfile >> spePdf.MaxDmin;
  myfile >> spePdf.MaxDmax;

  for (int z = 0; z < 22; ++z) {
    for (int x = 0; x < spePdf.MaxVbins; ++x) {
      for (int y = 0; y < spePdf.MaxDbins; ++y) {
        myfile >> dummy;
        spePdf.MaxV_MaxD.push_back(dummy);
      }
    }
  }


  myfile.close();

  std::cout << "Read SmartQT configuration file okay" << std::endl;

  return true;
}

Byte_t v1720CONET2::EvalSQTPulse()
{
  Float_t charge = pulse->GetChargeADCRelBaseline();
  Float_t baseline = pulse->CalcBaselineADC();
  Float_t width = pulse->GetWidthBins();
  Float_t width_limit = (Float_t)(charge + pfConfig.MaxSPEWidthIntercept) * (pfConfig.MaxSPEWidthSlope);
  if (baseline > 3908 || baseline < 3892) return DS::QT::BAD_BASELINE;
  if (charge > pfConfig.MaxSPECharge) return DS::QT::CHARGE_TOO_HIGH;
  if (width > width_limit) return DS::QT::WIDTH_TOO_WIDE;
  if (charge < 25) return DS::QT::FAIL_LIKELIHOOD;

  // DIFF
  if (IsAdjacent) return DS::QT::ADJACENT_BLOCK;

  return (Byte_t)(200.*(GetBinMaxV_ChargeFrac()*GetBinMaxV_MaxD()));
}

//MaxVMaxD PDF value.
Float_t v1720CONET2::GetBinMaxV_MaxD()
{
  int graph = (int)((pulse->GetChargeADCRelBaseline() - 50.)/20.);
  if (graph < 0 || graph > 21) return 0.;
  int xbin = (pulse->GetChargeHeightRatio() - spePdf.MaxVmin)/((spePdf.MaxVmax-spePdf.MaxVmin)/spePdf.MaxVbins);
  if (xbin<0 || xbin >= spePdf.MaxVbins) return 0;
  int ybin = (pulse->GetChargeMaxDRatio() - spePdf.MaxDmin)/((spePdf.MaxDmax-spePdf.MaxDmin)/spePdf.MaxDbins);
  if (ybin<0 || ybin >= spePdf.MaxDbins) return 0;
  return spePdf.MaxV_MaxD[spePdf.MaxDbins*spePdf.MaxVbins*graph+spePdf.MaxDbins*xbin+ybin];
}

//MaxVChargeFrac PDF value.
Float_t v1720CONET2::GetBinMaxV_ChargeFrac()
{
  int graph = (int)((pulse->GetChargeADCRelBaseline() - 50.)/20.);
  if (graph < 0 || graph > 21) return 0.;
  int xbin = (pulse->GetChargeHeightRatio() - spePdf.MaxVmin)/((spePdf.MaxVmax-spePdf.MaxVmin)/spePdf.MaxVbins);
  if (xbin<0 || xbin >= spePdf.MaxVbins) return 0;
  int ybin = (pulse->GetChargeFrac() - spePdf.ChargeFracmin)/((spePdf.ChargeFracmax-spePdf.ChargeFracmin)/spePdf.ChargeFracbins);
  if (ybin<0 || ybin >= spePdf.ChargeFracbins) return 0;
  return spePdf.MaxV_ChargeFrac[spePdf.MaxVbins*spePdf.ChargeFracbins*graph+spePdf.ChargeFracbins*xbin+ybin];
}



/*
// Returns the PDF value for MaxV_ChargeFrac
double v1720CONET2::GetBinMaxV_ChargeFrac()
{
  int graph =(int)((Charge_conv - 50.)/20.);
  if (graph < 0 || graph > 21) return 0.;
  int xbin = (Charge_MaxV - MaxVmin)/((MaxVmax-MaxVmin)/MaxVbins);
  if (xbin<0 || xbin >= MaxVbins) return 0;
  int ybin = (Charge_Frac - ChargeFracmin)/((ChargeFracmax-ChargeFracmin)/ChargeFracbins);
  if (ybin<0 || ybin >= ChargeFracbins) return 0;
  return MaxV_ChargeFrac[MaxVbins*ChargeFracbins*graph+ChargeFracbins*xbin+ybin];
}

// Returns the PDF value for MaxV_MaxD
double v1720CONET2::GetBinMaxV_MaxD()
{
  int graph =(int)((Charge_conv - 50.)/20.);
  if (graph < 0 || graph > 21) return 0.;
  int xbin = (Charge_MaxV - MaxVmin)/((MaxVmax-MaxVmin)/MaxVbins);
  if (xbin<0 || xbin >= MaxVbins) return 0;
  int ybin = (Charge_MaxD - MaxDmin)/((MaxDmax-MaxDmin)/MaxDbins);
  if (ybin<0 || ybin >= MaxDbins) return 0;
  return MaxV_MaxD[MaxDbins*MaxVbins*graph+MaxDbins*xbin+ybin];
}

// ResetPulseVariables resets pulse variables back to null values.
void v1720CONET2::ResetPulseVariables()
{
  BaselineBefore = 0;
  Charge = 0;
  MaxD = 0;
  PrePreMaxV = 0;
  PreMaxV = 0;
  PostMaxV = 0;
  PeakVoltage = 3900;
  PeakTime = 0;
  Start = 0;
  Stop = 0;
  Conf = 0;
  ChargePeak=0;
}
*/
//
//--------------------------------------------------------------------------------
/**
 * \brief   Fill Smart QT Bank
 *
 * \param   [in]  pevent  pointer to event buffer
 * \param   [in]  pZLEData  pointer to the data area of the bank
 */
bool v1720CONET2::FillSmartQTBank(char * pevent, uint32_t * pZLEData){

  DWORD *src;
  DWORD *dest;
  uint32_t nQTWords;

  //The read pointer points to the QT data following ZLE Event data
  int status = rb_get_rp(this->GetRingBufferHandle(), (void**)&src, 100);
  if (status == DB_TIMEOUT) {
    cm_msg(MERROR,"FillSmartQTBank", "Got rp timeout for module %d", this->GetModuleID());
    printf("rp timeout, Module %d\n", this->GetModuleID());
    return false;
  }

  char tBankName[5];
  snprintf(tBankName, sizeof(tBankName), "SQ%02d",this->GetModuleID());
  bk_create(pevent, tBankName, TID_DWORD, (void **)&dest);

  //Copy Smart QT header
  memcpy(dest, src, 3*sizeof(uint32_t));  //The Smart QT header is 3 words long (see ReadSmartQTData())
  nQTWords = *(src + 2);
  //if (this->GetModuleID() == 0) printf("### FillSmartQTBank: nQTWords: %u\n", nQTWords);

  //Copy Smart QT words
  memcpy(dest+3, src+3, nQTWords*sizeof(uint32_t));

  rb_increment_rp(this->GetRingBufferHandle(), (3 + nQTWords)*sizeof(uint32_t));
  bk_close(pevent, dest + 3 + nQTWords);

  return true;
}


//
//--------------------------------------------------------------------------------
/**
 * \brief   Fill Minima Bank
 *
 * \param   [in]  pevent  pointer to event buffer
 * \param   [in]  pZLEData  pointer to the data area of the bank
 */
bool v1720CONET2::FillMinimaBank(char * pevent, uint32_t * pZLEData){

  DWORD *src;
  DWORD *dest;

  //The read pointer points to the QT data following ZLE Event data
  int status = rb_get_rp(this->GetRingBufferHandle(), (void**)&src, 100);
  if (status == DB_TIMEOUT) {
    cm_msg(MERROR,"FillMinimaBank", "Got rp timeout for module %d", this->GetModuleID());
    printf("rp timeout, Module %d\n", this->GetModuleID());
    return false;
  }

  char tBankName[5];
  snprintf(tBankName, sizeof(tBankName), "MN%02d",this->GetModuleID());
  bk_create(pevent, tBankName, TID_DWORD, (void **)&dest);

  // Only 6 words in this bank - see ReadMinimaData()
  memcpy(dest, src, 6*sizeof(uint32_t));

  rb_increment_rp(this->GetRingBufferHandle(), 6*sizeof(uint32_t));
  bk_close(pevent, dest + 6);

  return true;
}


//
//--------------------------------------------------------------------------------
bool v1720CONET2::FillQTBankOld(char * pevent, uint32_t * pZLEData){
  // >>> Create bank.
  // QtData is the pointer to the memory location hodling the Qt data
  // Content
  // V1720 event counter
  // V1720 Trigger time tag
  // Number of Qt
  // QT format: channel 28 time 16 charge 0
  char tBankName[5];
  snprintf(tBankName, sizeof(tBankName), "QT%02d",this->moduleID_);
  uint32_t *QtData;
  bk_create(pevent, tBankName, TID_DWORD, (void **)&QtData);


  // >>> copy some header words
  *QtData = *(pZLEData+2); // event counter QtData[0]
  QtData++;
  *QtData = *(pZLEData+3); // trigger time tag QtData[1]
  QtData++;

  // >>> Figure out channel mapping
  //if(mNCh==0){
  int mNCh=0;
  uint32_t mChMap[8];
  uint32_t chMask = pZLEData[1] & 0xFF;
  for(int iCh=0; iCh<8; iCh++){
    if(chMask & (1<<iCh)){
      mChMap[mNCh] = iCh;
      mNCh++;
    }
  }
  if(mNCh==0){
    // printf("No channels found for module %i! Something wrong with channel mask\n", aModule);
  }

  // >>> Skip location QtData[2]. Will be used for number of QT;
  uint32_t* nQt = QtData;
  *(nQt) = 0;
  QtData++;
  //std::cout << QtData[0] <<  " " << *(QtData-2) << " " << QtData[1] << " " << *nQt << " " << *(QtData-1)<< " ";
  // >>> Loop over ZLE data and fill up Qt data bank

  uint32_t iPtr=4;
  for(int iCh=0; iCh<mNCh; iCh++){
    uint32_t chSize = pZLEData[iPtr];
    uint32_t iChPtr = 1;// The chSize space is included in chSize
    uint32_t iBin=0;
    iPtr++;
    //std::cout << "--------------- Channel: " << aModule << "-" << iCh << " size=" << chSize << " " <<  std::endl;
    uint32_t prevGoodData=1;
    while(iChPtr<chSize){
      uint32_t goodData = ((pZLEData[iPtr]>>31) & 0x1);
      uint32_t nWords = (pZLEData[iPtr] & 0xFFFFF);
      if(prevGoodData==0 && goodData==0){ // consecutive skip. Bad
        /*  std::cout << "consecutive skip: V1720=" << aModule
      << " | ch=" << iCh
      << " | prev word="  << pZLEData[iPtr-1]
      << " | cur word=" << pZLEData[iPtr] << std::endl;
         */}
      prevGoodData=goodData;
      if(goodData){
        uint32_t iMin = iBin;
        uint32_t min=4096;
        uint32_t baseline = (pZLEData[iPtr+1]&0xFFF);
        for(uint32_t iWord=0; iWord<nWords; iWord++){
          iPtr++;
          iChPtr++;
          if(min > (pZLEData[iPtr]&0xFFF) ){
            iMin = iBin+iWord*2;
            min = (pZLEData[iPtr]&0xFFF);
          }
          if(min > ((pZLEData[iPtr]>>16)&0xFFF) ){
            iMin = iBin+iWord*2+1;
            min = ((pZLEData[iPtr]>>16)&0xFFF);
          }
        }
        // package channel | iMin | min in one 32 bit word
        // don't bother for now. Temporary!!!!
        (*nQt)++; // increment number of Qt
        min = baseline-min; // turn it into a positive number
        *QtData = (((mChMap[iCh]<<28) & 0xF0000000) |
            ((iMin<<16)        & 0x0FFF0000) |
            (min               & 0x0000FFFF));
        QtData++;
        //std::cout << aModule << " "  << mChMap[iCh] << " " << iMin << " " << min << " " << *(QtData-1) << std::endl;
      }
      else{  // skip
        iBin += (nWords*2);
      }

      iChPtr++;
      iPtr++;
    }
  }
  //std::cout << " | " << *nQt << std::endl;
  bk_close(pevent, QtData);

  return true;
}

//
//--------------------------------------------------------------------------------
bool v1720CONET2::FillStatBank(char * pevent, suseconds_t usStart)
{
  if (! this->IsConnected()) {
    cm_msg(MERROR,"FillStatBank","Board %d disconnected", this->GetModuleID());
    return false;
  }

  DWORD *pdata, eStored, eSize;
  CAENComm_ErrorCode sCAEN;

  sCAEN = ReadReg_(V1720_EVENT_STORED, &eStored);
  sCAEN = ReadReg_(V1720_EVENT_SIZE, &eSize);

  // >>> Statistical bank for data throughput analysis
  char statBankName[5];
  snprintf(statBankName, sizeof(statBankName), "ST%02d", this->GetModuleID());
  bk_create(pevent, statBankName, TID_DWORD, (void **)&pdata);
  *pdata++ = this->GetModuleID();
  *pdata++ = eStored;
  *pdata++ = eSize;
  *pdata++ = usStart;               //time before read
  timeval tv; gettimeofday(&tv,0);
  *pdata++ = tv.tv_usec;            //time after read
  bk_close(pevent, pdata);

  return (sCAEN == CAENComm_Success);
}

//
//--------------------------------------------------------------------------------
bool v1720CONET2::FillBufferLevelBank(char * pevent)
{
  if (! this->IsConnected()) {
    cm_msg(MERROR,"FillBufferLevelBank","Board %d disconnected", this->GetModuleID());
    return false;
  }

  DWORD *pdata, eStored, almostFull;
  int rb_level;
  char statBankName[5];
  CAENComm_ErrorCode sCAEN;

  snprintf(statBankName, sizeof(statBankName), "BL%02d", this->GetModuleID());
  bk_create(pevent, statBankName, TID_DWORD, (void **)&pdata);

  //Get v1720 buffer level
  sCAEN = ReadReg_(V1720_EVENT_STORED, &eStored);
  sCAEN = ReadReg_(V1720_ALMOST_FULL_LEVEL, &almostFull);
  //Get ring buffer level
  rb_get_buffer_level(this->GetRingBufferHandle(), &rb_level);

  *pdata++ = eStored;
  /***
   * Note: There is no register in the v1720 indicating a busy
   * signal being output.  So we have to deduce it from the buffer
   * level and the almost full setting
   */
  int busy = 0;
  if(almostFull == 0){
    /* If the almost full register is set to 0,
     * the busy signal comes out only when all the
     * 1024 buffers are used */
    busy = (eStored == 1024) ? 1 : 0;
  }
  else {
    busy = (eStored >= almostFull) ? 1 : 0;
  }
  *pdata++ = busy*500; //Make it 500 for better histogram display

  *pdata++ = rb_level;

  if(busy)
    printf(" %d(B)/%u ", eStored, rb_level);
  else
    printf(" %d/%u", eStored, rb_level);


  bk_close(pevent, pdata);


  return (sCAEN == CAENComm_Success);

}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Send a software trigger to the board
 *
 * Send a software trigger to the board.  May require
 * software triggers to be enabled in register 0x810C.
 *
 * \return  CAENComm Error Code (see CAENComm.h)
 */
bool v1720CONET2::SendTrigger()
{
  if (verbosity_) std::cout << GetName() << "::SendTrigger()" << std::endl;
  if (!IsConnected()) {
    cm_msg(MERROR,"SendTrigger","Board %d disconnected", this->GetModuleID());
    return false;
  }

#if SIMULATION
  if (verbosity_) std::cout << "Sending Trigger " << moduleID_ << std::endl;
#else
  if (verbosity_) std::cout << "Sending Trigger (l,b) = (" << link_ << "," << board_ << ")" << std::endl;
#endif

  return (WriteReg(V1720_SW_TRIGGER, 0x1) == CAENComm_Success);
}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Set the ODB record for this board
 *
 * Create a record for the board with settings from the configuration
 * string (v1720CONET2::config_str_board) if it doesn't exist or merge with
 * existing record. Create hotlink with callback function for when the
 * record is updated.  Get the handle to the record.
 *
 * Ex: For a frontend with index number 2 and board number 0, this
 * record will be created/merged:
 *
 * /Equipment/FEV1720I2/Settings/Board0
 *
 * \param   [in]  h        main ODB handle
 * \param   [in]  cb_func  Callback function to call when record is updated
 * \return  ODB Error Code (see midas.h)
 */
int v1720CONET2::SetBoardRecord(HNDLE h, void(*cb_func)(INT,INT,void*))
{
  char set_str[200];

#if SIMULATION
  snprintf(set_str, sizeof(set_str), "/Equipment/FEV1720MT_SIM/Settings/Board%d", moduleID_);
#else
  if(feIndex_ == -1)
    snprintf(set_str, sizeof(set_str), "/Equipment/FEV1720MTI/Settings/Board%d", moduleID_ % 8);
  else
    snprintf(set_str, sizeof(set_str), "/Equipment/FEV1720MTI%02d/Settings/Board%d", feIndex_, moduleID_ % 8);
#endif

  if (verbosity_) std::cout << GetName() << "::SetBoardRecord(" << h << "," << set_str << ",...)" << std::endl;
  int status,size;
  //create record if doesn't exist and find key
  status = db_create_record(h, 0, set_str, strcomb(config_str_board));
  status = db_find_key(h, 0, set_str, &settings_handle_);
  if (status != DB_SUCCESS) {
    cm_msg(MINFO,"SetBoardRecord","Key %s not found. Return code: %d", set_str, status);
  }

  //hotlink
  size = sizeof(V1720_CONFIG_SETTINGS);
  status = db_open_record(h, settings_handle_, &config, size, MODE_READ, cb_func, NULL);
  if (status != DB_SUCCESS){
    cm_msg(MERROR,"SetBoardRecord","Couldn't create hotlink for %s. Return code: %d", set_str, status);
    return status;
  }

  //get actual record
  status = db_get_record(h, settings_handle_, &config, &size, 0);
  if (status != DB_SUCCESS){
    cm_msg(MERROR,"SetBoardRecord","Couldn't get record %s. Return code: %d", set_str, status);
    return status;
  }

  settings_loaded_ = true;
  settings_touched_ = true;

  return status; //== DB_SUCCESS for success
}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Set the ODB record for history variable names
 *
 * \param   [in]  h        main ODB handle
 * \param   [in]  cb_func  Callback function to call when record is updated
 * \return  ODB Error Code (see midas.h)
 */
int v1720CONET2::SetHistoryRecord(HNDLE h, void(*cb_func)(INT,INT,void*))
{
  char settings_path[200] = "/Equipment/BUFLVLMT/Settings/";
  char names_path[200];

#if SIMULATION
  snprintf(settings_path, sizeof(settings_path), "/Equipment/BUFLVLMT_SIM/Settings/");
#else
  if(feIndex_ == -1)
    snprintf(settings_path, sizeof(settings_path), "/Equipment/BUFLVLMT/Settings/");
  else
    snprintf(settings_path, sizeof(settings_path), "/Equipment/BUFLVLMT%02d/Settings/", feIndex_);
#endif

//  if (verbosity_) std::cout << GetName() << "::SetHistoryRecord(" << h << "," << settings_path << ",...)" << std::endl;
  int status;//,size;

  HNDLE settings_key;
  status = db_find_key(h, 0, settings_path, &settings_key);

  if(status == DB_NO_KEY){
    db_create_key(h, 0, settings_path, TID_KEY);
    db_find_key(h, 0, settings_path, &settings_key);
  }

  char tmp[11];
  snprintf(tmp, sizeof(tmp), "Names BL%02d", this->moduleID_);
  strncpy(names_path, settings_path, sizeof(names_path));
  strncat(names_path, tmp, sizeof(names_path));

  db_create_key(h, 0, names_path, TID_STRING);
  HNDLE path_key;
  status = db_find_key(h, 0, names_path, &path_key);

  db_set_data(h, path_key, history_settings, sizeof(history_settings),
      sizeof(history_settings)/NAME_LENGTH, TID_STRING);

  if (status != DB_SUCCESS) cm_msg(MINFO,"SetHistoryRecord","Key %s not found", names_path);
  return status;
}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Initialize the hardware for data acquisition
 *
 * ### Initial setup:
 * - Set FP I/O Ctrl (0x811C) to default settings (output trigger).
 * - Do software reset + clear.
 * - Set up busy daisy chaining
 * - Put acquisition on stop.
 *
 * ### Checks
 * - AMC firmware version (and check that each channel has the same AMC firmware)
 * - ROC firmware version
 * - board type
 *
 * ### Set registers
 * Use a preset if setup != 0 in the config string, or set them manually otherwise.
 *
 * \return  0 on success, -1 on error
 */
int v1720CONET2::InitializeForAcq()
{
  if (verbosity_) std::cout << GetName() << "::InitializeForAcq()" << std::endl;

  if (!settings_loaded_) {
    cm_msg(MERROR,"InitializeForAcq","Cannot call InitializeForAcq() without settings loaded properly on board %d", this->GetModuleID());
    return -1;
  }
  if (!IsConnected()){
    cm_msg(MERROR,"InitializeForAcq","Board %d disconnected", this->GetModuleID());
    return -1;
  }
  if (IsRunning()){
    cm_msg(MERROR,"InitializeForAcq","Board %d already started", this->GetModuleID());
    return -1;
  }
        

	// TL change: always do the board re-initialization.
  //if (settings_loaded_ && !settings_touched_)
	//return 0;
        
  CAENComm_ErrorCode sCAEN;
        
  // Do special board reset for lockup prevention                                                 
	DWORD reg;
  // Clear the board                                                         
	sCAEN = WriteReg_(V1720_SW_RESET, 0x1);
	
	// Need time for the PLL to lock
	ss_sleep(500);
	
	// Clear done by accessing Buffer Origanization later on
	//  sCAEN = WriteReg_(V1720_SW_CLEAR, 0x1);
  
  // Set register V1720_FP_IO_CONTROL (0x811C) to default settings
  // (output trigger) will set the board that output the clock latter
  sCAEN = WriteReg_(V1720_FP_IO_CONTROL, 0x00000000);
        
  // Setup Busy daisy chaining
  sCAEN = WriteReg_(V1720_FP_IO_CONTROL,        0x104); // 0x100:enable new config, 0x4:LVDS I/O[3..0] output
  sCAEN = WriteReg_(V1720_FP_LVDS_IO_CRTL,      0x022); // 0x20: I/O[7..4] input mode nBusy/Veto (4:nBusy input)
                                                        // 0x02: I/O[3..0] output mode nBusy/Veto (1:Busy)

  std::stringstream ss_fw_datatype;
  ss_fw_datatype << "Module " << moduleID_ << ", ";
#if SIMULATION
  cm_msg(MINFO,"InitializeForAcq","Simulation, no firmware check");
#else
  // Firmware version check
  // read each AMC firmware version
  // [31:16] Revision date Y/M/DD
  // [15:8] Firmware Revision (X)
  // [7:0] Firmware Revision (Y)
  // eg 0x760C0103 is 12th June 07, revision 1.3
  int addr = 0;
  uint32_t version = 0;
  uint32_t prev_chan = 0;
  // Hardcode correct firmware verisons
  // Current release 3.4_0.11 Feb 2012
  //  const uint32_t amc_fw_ver = 0xd215000c;
  //  const uint32_t amc_fw_ver = 0xe527000d;
  //  const uint32_t roc_fw_ver = 0xc2080304;
  //  const uint32_t roc_fw_ver = 0xd4110401;
  //  const uint32_t roc_fw_ver = 0xe5270402;
  const uint32_t roc_fw_ver_test = 0xc5250306;   //new version we're testing
   const uint32_t amc_fw_ver = 0xe904000d; // Update 22Oct2014
   const uint32_t roc_fw_ver = 0xe9100403; // Update 22Oct2014
  for(int iCh=0;iCh<8;iCh++) {
    addr = 0x108c | (iCh << 8);
    sCAEN = ReadReg_(addr, &version);
    if((iCh != 0) && (prev_chan != version)) {
      cm_msg(MERROR, "InitializeForAcq","Error Channels have different AMC Firmware ");
    }
    prev_chan = version;
  }
        //  cm_msg(MINFO,"feoV1720","Format: YMDD:XX.YY");
  if(version != amc_fw_ver)
    cm_msg(MERROR,"InitializeForAcq","Incorrect AMC Firmware Version: 0x%08x, 0x%08x expected", version, amc_fw_ver);
  else
    ss_fw_datatype << "AMC FW: 0x" << std::hex << version << ", ";

  // read ROC firmware revision
  // Format as above
  sCAEN = ReadReg_(V1720_ROC_FPGA_FW_REV, &version);
  switch (version)
  {
  case roc_fw_ver:
    ss_fw_datatype << "ROC FW: 0x" << std::hex << version << ", ";
    break;
  case roc_fw_ver_test:
    cm_msg(MINFO,"InitializeForAcq","*** WARNING *** using new ROC Firmware Version: 0x%08x", version);
    break;
  default:
    cm_msg(MERROR,"InitializeForAcq","Incorrect ROC Firmware Version: 0x%08x, 0x%08x expected", version, roc_fw_ver);
    break;
  }

  // Verify Board Type
  const uint32_t v1720_board_type = 0x03;
  sCAEN = ReadReg_(V1720_BOARD_INFO, &version);
  if((version & 0xFF) != v1720_board_type)
    cm_msg(MINFO,"InitializeForAcq","*** WARNING *** Trying to use a v1720 frontend with another"
        " type of board.  Results will be unexpected!");

#endif  //SIMULATION

//  ss_fw_datatype << this->GetChannelConfig();
  switch(this->GetDataType()){
  case RawPack2:
    ss_fw_datatype << "Raw Data";
    break;
  case RawPack25:
    ss_fw_datatype << "Raw Data 2.5 Packing";
    break;
  case ZLEPack2:
    ss_fw_datatype << "ZLE Data";
    break;
  case ZLEPack25:
    ss_fw_datatype << "ZLE Data 2.5 Packing";
    break;
  case UnrecognizedDataFormat:
    ss_fw_datatype << "Unrecognized data format";
    break;
  default:
    /* Can't happen */
    break;
  }

        //-PAA- Moved after PLL check
        //  cm_msg(MINFO, "InitializeForAcq", ss_fw_datatype.str().c_str());

  //use preset setting if enabled
  if (config.setup != 0) SetupPreset_(config.setup);
  //else use odb values
  else
  {
    //already reset/clear earlier this function, so skip here
    AcqCtl_(config.acq_mode);
    WriteReg_(V1720_CHANNEL_CONFIG,          config.channel_config);
    WriteReg_(V1720_BUFFER_ORGANIZATION,     config.buffer_organization);
    WriteReg_(V1720_CUSTOM_SIZE,             config.custom_size);

    /* A bug exists in the firmware where if the channel mask is 0 (all channels
     * disabled), the board misbehaves (reports bogus number of events in output
     * buffer, event ready register doesn't work, etc).  Don't allow it     */
    if(!config.channel_mask){
      cm_msg(MERROR,"InitializeForAcq","The board misbehaves if channel mask is 0 (all channels disabled). Exiting...");
      exit(FE_ERR_HW);
    }

    WriteReg_(V1720_CHANNEL_EN_MASK,         config.channel_mask);
    AcqCtl_(V1720_COUNT_ACCEPTED_TRIGGER);
    WriteReg_(V1720_TRIG_SRCE_EN_MASK,       config.trigger_source);
    WriteReg_(V1720_FP_TRIGGER_OUT_EN_MASK,  config.trigger_output);
    WriteReg_(V1720_POST_TRIGGER_SETTING,    config.post_trigger);
    WriteReg_(V1720_ALMOST_FULL_LEVEL,       config.almost_full);
    WriteReg_(V1720_MONITOR_MODE,            0x3);
    WriteReg_(V1720_BLT_EVENT_NB,            0x1);
    WriteReg_(V1720_VME_CONTROL,             V1720_ALIGN64);

			{
				// Change Clock signal current on the Clock Cleaner from
				// (setting=4 for 5.25mA to setting=6 for 6mA)
				// Read current setting 
				DWORD iClock;
				WriteReg_(V1720_AD9510, 0x804200);     // Read mode
				ReadReg_(V1720_AD9510, &iClock);
				printf("AD9510 current mode: %d\n", iClock);
				WriteReg_(V1720_AD9510, 0x004206);     // Write mode=6
				WriteReg_(V1720_AD9510, 0x005A01);  // udpate AD9510
				WriteReg_(V1720_AD9510, 0x804200);     // Read mode
				ReadReg_(V1720_AD9510, &iClock);
				printf("AD9510 current mode: %d\n", iClock);
			}

    //set specfic channel values
    for (int iChan=0; iChan<8; iChan++)
    {
      WriteReg_(V1720_CHANNEL_THRESHOLD   + (iChan<<8), config.auto_trig_threshold     [iChan]);
      WriteReg_(V1720_CHANNEL_OUTHRESHOLD + (iChan<<8), config.auto_trig_N_4bins_min [iChan]);
//      _WriteReg(V1720_ZS_THRESHOLD        + (iChan<<8), config.zs_threshold  [iChan]);
      // !!!!!!! If this disapears one more time, I will be very pissed off. FR
      if( config.zle_signed_threshold[iChan]>0){
        WriteReg_(V1720_ZS_THRESHOLD        + (iChan<<8), config.zle_signed_threshold  [iChan]);
      }
      else{// this is necessary for negative going pulses. It is easier to use decimal numbers
        WriteReg_(V1720_ZS_THRESHOLD        + (iChan<<8), (0x80000000 | (-1*config.zle_signed_threshold[iChan])));
      }
//      _WriteReg(V1720_ZS_NSAMP            + (iChan<<8), config.zs_nsamp      [iChan]);
      WriteReg_(V1720_ZS_NSAMP            + (iChan<<8), ((config.zle_bins_before[iChan]<<16) | config.zle_bins_after[iChan]));
      WriteReg_(V1720_CHANNEL_DAC         + (iChan<<8), config.dac           [iChan]);
      //NB: in original frontend, zst and zsn regs were set via a short calculation in the
      //frontend, not the exact values as in odb. it was not clear that this was done
      //without looking at source code, so i have changed it to just take values direct
      //from ODB.
      //On my todo list is to figure out a better way of setting up ODB settings
      //for the frontends.

    }

//    ov1720_Status(GetDeviceHandle());
  }

  //todo: some kind of check to make sure stuff is set correctly???
        // Check finally for Acquisition status
//      sCAEN = ReadReg_(V1720_ACQUISITION_STATUS, &reg);
//      cm_msg(MINFO, "AcqInit", "Module %d (Link %d Board %d) Acquisition Status : 0x%x", moduleID_, link_, board_, reg);
        sCAEN = ReadReg_(V1720_ACQUISITION_STATUS, &reg);
        ss_fw_datatype << ", Acq Reg: 0x" << std::hex << reg;
        cm_msg(MINFO, "InitializeForAcq", ss_fw_datatype.str().c_str());

//      cm_msg(MINFO, "AcqInit", "Module %d (Link %d Board %d) Acquisition Status : 0x%x", moduleID_, link_, board_, reg);
        if ((reg & 0xF0) != 0xA0) {
                cm_msg(MERROR, "InitAcq", "Module %d (Link %d Board %d ) not initilized properly acq status:0x%x",  moduleID_, link_, board_, reg);
                return -1;
        }
/*
  // Read the DAC values, so we can print them to log for debugging purposes.
  std::stringstream ss_dac;
  ss_dac << "Module " << moduleID_ << ", DAC values are ";

  for (int iChan=0; iChan<8; iChan++) {
    DWORD dac_read = 0;
    sCAEN = ReadReg_(V1720_CHANNEL_DAC + (iChan<<8), &dac_read);
    ss_dac << dac_read << " ";
  }

  cm_msg(MINFO, "InitializeForAcq", ss_dac.str().c_str());
  // End of DAC debug code.
*/

  settings_touched_ = false;
  UNUSED(sCAEN);

  //ready to do start run

  return 0;
}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Get data type and ZLE configuration
 *
 * Finds the data type (raw/zle, pack2/2.5) from the channel
 * configuration (0x8000)
 *
 */
v1720CONET2::DataType v1720CONET2::GetDataType()
{
        
  // Set Device, data type and packing for QT calculation later
  int dataType = ((config.channel_config >> 11) & 0x1);
  if(((config.channel_config >> 16) & 0xF) == 0) {
    if(dataType == 1) {
      // 2.5 pack, full data
      data_type_ = RawPack25;
      return RawPack25;
    } else {
      // 2 pack, full data
      data_type_ = RawPack2;
      return RawPack2;
    }
  } else if(((config.channel_config >> 16) & 0xF) == 2) {
    if(dataType == 1) {
      // 2.5 pack, ZLE data
      data_type_ = ZLEPack25;
      return ZLEPack25;
    } else {
      // 2 pack, ZLE data
      data_type_ = ZLEPack2;
      return ZLEPack2;
    } 
        } else
          return UnrecognizedDataFormat;
}

//
//--------------------------------------------------------------------------------
/**
 * \brief   Get ZLE setting
 *
 * Get the current ZLE setting from the channel configuration.
 *
 * \return  true if data is ZLE
 */
bool v1720CONET2::IsZLEData(){
  return ((data_type_ == ZLEPack2)||(data_type_ == ZLEPack25));
}

/* emacs
 * Local Variables:
 * mode:C
 * mode:font-lock
 * tab-width: 2
 * c-basic-offset: 2
 * End:
 */


