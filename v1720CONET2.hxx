/*****************************************************************************/
/**
\file v1720CONET2.hxx

## Contents

This file contains the class definition for the v1720 module driver.
 *****************************************************************************/

#ifndef V1720_HXX_INCLUDE
#define V1720_HXX_INCLUDE

#include <iostream>
#include <sstream>
#include <iomanip>
#include <assert.h>
#include <stdlib.h>
#include <sys/time.h>
#include <atomic>
#include <vector>

extern "C" {
#include <CAENComm.h>
#include <CAENVMElib.h>
#include "ov1720drv.h"
}
#include "midas.h"
#include "msystem.h"

typedef unsigned short UShort_t;    //Unsigned Short integer 2 bytes (unsigned short)
typedef short          Short_t;     //Signed Short integer 2 bytes (unsigned short)
typedef float          Float_t;     //Float 4 bytes (float)
typedef unsigned char  Byte_t;      //Byte (8 bits) (unsigned char)
typedef unsigned int   UInt_t;      //Unsigned integer 4 bytes (unsigned int)
typedef int            Int_t;       //Signed integer 4 bytes (int)
typedef bool           Bool_t;      //Boolean (0=false, 1=true) (bool)

namespace TMath {
  inline Int_t Abs(Int_t d)
    { return std::abs(d); }

  inline Int_t Max(Int_t a, Int_t b)
     { return a >= b ? a : b; }
};

namespace DS {
  namespace QT {
    enum SpeFailureReasons {
      FAIL_LIKELIHOOD = 0, ///< Pulse is "sensible", but not SPE-like.
      CHARGE_TOO_HIGH = 248, ///< Integrated charge too high to be SPE-like.
      FEW_BASELINE_SAMPLES = 249, ///< Fewer than 5 baseline samples - likely innaccurate.
      WIDTH_TOO_WIDE = 250, ///< Pulse was too wide to be SPE-like.
      FINISHES_TOO_LATE = 251, ///< Pulse finished too close to end of ZLE block - may be truncated.
      CHARGE_EXTREMELY_HIGH = 252, ///< Integrated charge > 60000ADC, may overflow UShort_t.
      PEAK_TOO_SHARP = 253, ///< Pre-pre/pre/post sample too different from minimum sample, may overflow Byte_t.
      BAD_BASELINE = 254, ///< Baseline outside range 3892-3908. Cannot be accurately stored.
      ADJACENT_BLOCK = 255 ///< Two ZLE blocks are adjacent, may have a pulse split across.
    };
  }
}

namespace PulseUtil {
  enum PulseType {
    TYPE_V1720, ///< The pulse came from a V1720 board
    TYPE_V1720_SMARTQT, ///< The pulse came from a V1720 board through front end SmartQT pulse finding.
    TYPE_V1740, ///< The pulse came from a V1740 board
    TYPE_COUNT  ///< Do not add anything after TYPE_COUNT
  };
}

namespace QTUtil {
  inline Float_t CalcChargeMaxDRatio(Float_t ChargeConv,Short_t MaxD)
  {
    return -ChargeConv/MaxD;
  }

  inline Float_t CalcChargeHeightRatio(Float_t ChargeConv,Float_t Baseline,UShort_t MaxV)
  {
    return ChargeConv/(Baseline - (Float_t)MaxV);
  }

  inline Float_t CalcChargeFrac(Float_t ChargeConv,Float_t Baseline,UShort_t maxVT, UShort_t left, UShort_t chargetopeakADC)
  {
    Float_t Charge_Frac = -((4096.-Baseline)*(Float_t)(maxVT-left+1) - (Float_t)chargetopeakADC);
    Charge_Frac /= (ChargeConv-Charge_Frac);
    return Charge_Frac;
  }
}

class Pulse {
  public:
    Pulse() {
      chargeADC = 0;
      minimumADC = 0;
      offset = 0;
      left = 0;
      right = 0;
      baselineIntRel3892 = 0;
      baselineSquareIntRel3892 = 0;
      baselineSamples = 0;
      for (int i = 0; i < 4; i++) {
        SubnsSamples[i] = 0;
      }
      peakT = 0;
      conf = 0;
      type = PulseUtil::TYPE_COUNT;

      peakchargeADC = 0;
      maxD = 0;
      maxV = 0;
      maxVT = 0;
    }

    ~Pulse() {}

    Pulse(Pulse&& other) noexcept {
      chargeADC = std::move(other.chargeADC);
      minimumADC = std::move(other.minimumADC);
      offset = std::move(other.offset);
      left = std::move(other.left);
      right = std::move(other.right);
      baselineIntRel3892 = std::move(other.baselineIntRel3892);
      baselineSquareIntRel3892 = std::move(other.baselineSquareIntRel3892);
      baselineSamples = std::move(other.baselineSamples);
      for (int i = 0; i < 4; i++) {
        SubnsSamples[i] = std::move(other.SubnsSamples[i]);
      }
      peakT = std::move(other.peakT);
      conf = std::move(other.conf);
      type = std::move(other.type);

      peakchargeADC = std::move(other.peakchargeADC);
      maxD = std::move(other.maxD);
      maxV = std::move(other.maxV);
      maxVT = std::move(other.maxVT);
    }
    Pulse& operator=(Pulse&& other) noexcept {
      if (this != &other) {
        chargeADC = std::move(other.chargeADC);
        minimumADC = std::move(other.minimumADC);
        offset = std::move(other.offset);
        left = std::move(other.left);
        right = std::move(other.right);
        baselineIntRel3892 = std::move(other.baselineIntRel3892);
        baselineSquareIntRel3892 = std::move(other.baselineSquareIntRel3892);
        baselineSamples = std::move(other.baselineSamples);
        for (int i = 0; i < 4; i++) {
          SubnsSamples[i] = std::move(other.SubnsSamples[i]);
        }
        peakT = std::move(other.peakT);
        conf = std::move(other.conf);
        type = std::move(other.type);

        peakchargeADC = std::move(other.peakchargeADC);
        maxD = std::move(other.maxD);
        maxV = std::move(other.maxV);
        maxVT = std::move(other.maxVT);
      }
      return *this;
    }


    void SetChargeADCRel4096(UShort_t _charge) { chargeADC = _charge;}
    UShort_t GetChargeADCRel4096() { return chargeADC;}

    Float_t GetChargeADCRelBaseline()
    {
      return -((4096.-CalcBaselineADC())*((Float_t)GetWidthBins()) - (Float_t)chargeADC);
    }

    Float_t CalcBaselineADC()
    {
      if (baselineSamples == 0) {
        return 0;
      } else {
        return (3892. + ((Float_t)baselineIntRel3892/(Float_t)baselineSamples));
      }
    }

    UShort_t GetWidthBins() {return (right-left+1);}

    void SetMinimumADC(UShort_t _minimum) { minimumADC = _minimum;}
    UShort_t GetMinimumADC() { return minimumADC;}

    void SetOffset(UShort_t _offset) { offset = _offset; }
    UShort_t GetOffset() { return offset; }

    void SetLeftEdge(UShort_t _left) { left = _left;}
    int GetLeftEdge() { return (int)left; }

    void SetRightEdge(UShort_t _right) { right = _right;}
    int GetRightEdge() { return (int)right; }

    void SetBaselineIntRel3892(UShort_t _baselineIntRel3892) { baselineIntRel3892 = _baselineIntRel3892;}
    UShort_t GetBaselineIntRel3892() { return baselineIntRel3892; }

    void SetBaselineSquareIntRel3892(UInt_t _baselineSquareIntRel3892) { baselineSquareIntRel3892 = _baselineSquareIntRel3892;}
    UInt_t GetBaselineSquareIntRel3892() { return baselineSquareIntRel3892; }

    void SetBaselineSamples(Byte_t _baselineSamples) { baselineSamples = _baselineSamples; }
    Byte_t GetBaselineSamples() { return baselineSamples; }



    void SetPrePreSample(UShort_t adc) { SubnsSamples[0] = adc; }
    void SetPreSample(UShort_t adc) { SubnsSamples[1] = adc; }
    void SetPeakVoltage(UShort_t adc) { SubnsSamples[2] = adc; }
    void SetPostSample(UShort_t adc) { SubnsSamples[3] = adc; }
    UShort_t GetPrePreSample() { return SubnsSamples[0]; }
    UShort_t GetPreSample() { return SubnsSamples[1]; }
    UShort_t GetPeakVoltage() { return SubnsSamples[2]; }
    UShort_t GetPostSample() { return SubnsSamples[3]; }

    void SetPeakTime(UShort_t _peakT) { peakT = _peakT; };
    UShort_t GetPeakTime() { return peakT; }

    void SetConfidence(Byte_t _conf) { conf = _conf; };
    Byte_t GetConfidence() { return conf; }

    void SetPulseType(PulseUtil::PulseType _type) { type = _type; }
    PulseUtil::PulseType GetPulseType() { return type; }

    void SetMaxD(Float_t _maxD) {maxD = _maxD;}
    void SetMaxV(UShort_t _maxV) {maxV = _maxV;}
    void SetMaxVT(UShort_t _maxVT) {maxVT = _maxVT;}
    void SetPeakChargeADCRel4096(UShort_t _peakchargeADC) {peakchargeADC = _peakchargeADC;}


    Float_t GetChargeFrac()
    {
      return QTUtil::CalcChargeFrac(GetChargeADCRelBaseline(),CalcBaselineADC(),maxVT,left,peakchargeADC);
    }

    Float_t GetChargeMaxDRatio() {
      return QTUtil::CalcChargeMaxDRatio(GetChargeADCRelBaseline(),maxD);
    }

    Float_t GetChargeHeightRatio() {
      return QTUtil::CalcChargeHeightRatio(GetChargeADCRelBaseline(),CalcBaselineADC(),maxV);
    }



  private:
    UShort_t chargeADC;
    UShort_t minimumADC;
    UShort_t offset;
    UShort_t left;
    UShort_t right;
    UShort_t baselineIntRel3892;
    UInt_t baselineSquareIntRel3892;
    Byte_t baselineSamples;
    UShort_t SubnsSamples[4];
    UShort_t peakT;
    Byte_t conf;
    PulseUtil::PulseType type;
    UShort_t peakchargeADC;
    Float_t maxD;
    UShort_t maxV;
    UShort_t maxVT;
};

class PMT {
  public:
    PMT() {
      pulse.clear();
    };
    ~PMT() {
      for (int i = 0; i < GetPulseCount(); i++) {
        if (pulse[i]) delete pulse[i];
      }
    };
    PMT(PMT&& other) noexcept {
      pulse = std::move(other.pulse);
    }
    PMT& operator=(PMT&& other) noexcept {
      if (this != &other) {
        pulse = std::move(other.pulse);
      }
      return *this;
    }
    Pulse *GetPulse(Int_t i) const { return pulse[i]; }
    Pulse *GetLastPulse() const { return pulse[pulse.size()-1]; }
    Pulse *AddNewPulse()
    {
      Pulse *o = new Pulse();
      pulse.push_back(o);
      return o;
    }
    void AddPulse(Pulse *p) {
      pulse.push_back(p);
    }
    void RemovePulse(int ipulse) {
      delete pulse[ipulse];
      pulse.erase(pulse.begin()+ipulse);
    }
    int GetPulseCount() { return pulse.size(); }

    // Just so we don't have to comment out more code
    Pulse* AddNewLGPulse() { return NULL; }
    Pulse* GetLGPulse(Int_t i) { (void)i; return NULL; }
    Int_t GetLGPulseCount() { return 0; }

  private:
    std::vector<Pulse*> pulse;
};


/**
 * Driver class for the v1720 module using the CAEN CONET2 (optical) interface.
 * Contains all the methods necessary to:
 *
 * - Connect/disconnect the board through an optical connection
 * - Initialize the hardware (set the registers) for data acquisition
 * - Read and write to the ODB
 * - Poll the hardware and read the event buffer into a midas bank
 * - Handle ZLE data
 * - Send a software trigger to the board if desired
 */
class v1720CONET2
{

public:

  /* Enums/structs */
  enum ConnectErrorCode {
    ConnectSuccess,
    ConnectErrorCaenComm,
    ConnectErrorTimeout,
    ConnectErrorAlreadyConnected
  };
  enum DataType {
    RawPack2,                //!< 0: Full data, 2 packing
    RawPack25,               //!< 1: Full data, 2.5 packing
    ZLEPack2,                //!< 2: ZLE data, 2 packing
    ZLEPack25,               //!< 3: ZLE data, 2.5 packing
    UnrecognizedDataFormat
  };
  struct V1720_CONFIG_SETTINGS {
    INT       setup; //!< Initial board setup mode number
    INT       acq_mode;                //!< 0x8100@[ 1.. 0]
    DWORD     channel_config;          //!< 0x8000@[19.. 0]
    INT       buffer_organization;     //!< 0x800C@[ 3.. 0]
    INT       custom_size;             //!< 0x8020@[31.. 0]
    DWORD     channel_mask;            //!< 0x8120@[ 7.. 0]
    DWORD     trigger_source;          //!< 0x810C@[31.. 0]
    DWORD     trigger_output;          //!< 0x8110@[31.. 0]
    DWORD     post_trigger;            //!< 0x8114@[31.. 0]
    // Hard code the two fp_* settings to alway on (Alex 21/2/13)
    //    DWORD     fp_io_ctrl;        //!< 0x811C@[31.. 0]
    DWORD     almost_full;             //!< 0x816C@[31.. 0]
    //    DWORD     fp_lvds_io_ctrl;   //!< 0x81A0@[31.. 0]
    DWORD     auto_trig_threshold[8];  //!< 0x1n80@[11.. 0]
    DWORD     auto_trig_N_4bins_min[8];//!< 0x1n84@[11.. 0]
    INT       zle_signed_threshold[8]; //!< 0x1n24@[31.. 0]
    INT       zle_bins_before[8];      //!< 0x1n28@[31.. 16]
    INT       zle_bins_after[8];       //!< 0x1n28@[15.. 0]
    DWORD     dac[8];                  //!< 0x1n98@[15.. 0]
    BOOL      qt_bank;                 //!< Write the QT bank
    BOOL      minima_bank;             //!< Write the MN bank
    BOOL      smartqt_bank;            //!< Write the SQ bank
    INT       PulseDerivativeThreshold;//!< For Smart QT
    float     PulseStop_MaxHeight;     //!< For Smart QT
    INT       PulseStop_MaxDeriv;      //!< For Smart QT
    float     PulseExtend_MinHeight;   //!< For Smart QT
    float     MinPulseCharge;          //!< For Smart QT
    float     MaxPulseCharge;          //!< For Smart QT
    float     MaxWidthIntercept;       //!< For Smart QT
    float     MaxWidthSlope;           //!< For Smart QT
  } config; //!< instance of config structure

  /* Static */
  static const char *config_str_board[]; //!< Configuration string for this board
  static const char history_settings[][NAME_LENGTH];

  /* Constructor/Destructor */
#if SIMULATION
  v1720CONET2(int board, HNDLE hDB);
#else
  v1720CONET2(int feindex, int link, int board, int moduleID, HNDLE hDB);
  /* Use move instead of copy semantics as we only need one copy of each
   * object (C++11).  See notes in implementation. */
  v1720CONET2(v1720CONET2&&) noexcept;
  v1720CONET2& operator=(v1720CONET2&&) noexcept;
#endif
  ~v1720CONET2();

  /* Public methods */
  ConnectErrorCode Connect();
  ConnectErrorCode Connect(int, int);
  static void * connectThread(void *);
  struct thread_args {
    v1720CONET2 * v1720;
    CAENComm_ErrorCode * errcode;
    pthread_cond_t * cv;
  };
  std::string connectStatusMsg;
  bool Disconnect();
  bool StartRun();
  bool StopRun();
  bool IsConnected();
  bool IsRunning();
  bool ReadReg(DWORD, DWORD*);
  bool WriteReg(DWORD, DWORD);
  bool CheckEvent();
  bool ReadEvent(void *);
  bool ReadEventCAEN(void *);
  bool ReadQTData(uint32_t *);
  bool ReadSmartQTData(uint32_t *);
  bool ReadMinimaData(uint32_t *);
  bool FillEventBank(char *);
  bool FillQTBank(char * aDest, uint32_t * aZLEData);
  bool FillSmartQTBank(char * aDest, uint32_t * aZLEData);
  bool FillMinimaBank(char * aDest, uint32_t * aZLEData);
  bool FillQTBankOld(char * aDest, uint32_t * aZLEData);
  bool FetchHeaderNextEvent(uint32_t * header);
  bool DeleteNextEvent();
  bool FillStatBank(char *, suseconds_t);
  bool FillBufferLevelBank(char *);
  bool IsZLEData();

  bool SendTrigger();
  bool Poll(DWORD*);
  int SetBoardRecord(HNDLE h, void(*cb_func)(INT,INT,void*));
  int SetHistoryRecord(HNDLE h, void(*cb_func)(INT,INT,void*));
  int InitializeForAcq();

  void FindPulses(bool firstBlock, int closePulseLookAhead, int closePulseNearEndOfBlock, bool extendPulseEnd, bool enableSubPeaks, bool enablePulseSplitting);
  void CalculateDerivativesFromSamples();
  void ClosePulseAndSetVariables(int& i, bool firstBlock, bool extendPulseEnd, int closePulseLookAhead);
  void SetPulseVariables();
  void ResetForNewChannel();
  void ResetForNewPulse();
  void ConvertPMTsToBanks(uint32_t*& QTData, uint32_t*& nQTWords);
  void UpdateBaselinesAndCutOnCharge(int chargeCutADCRelBaseline, bool lowGain = false);
  void AdjustPulseChargeAndTypeToAvoidOverflow();
  bool LoadSmartQTConfig();
  uint8_t EvalSQTPulse();
  Float_t GetBinMaxV_MaxD();
  Float_t GetBinMaxV_ChargeFrac();

  /* Getters/Setters */
#if SIMULATION
  int GetBoard() { return board_; }       //!< returns board number
  int GetModuleID() { return board_; }    //!< returns board number
#else
  int GetModuleID() { return moduleID_; } //!< returns unique module ID
  int GetLink() { return link_; }         //!< returns optical link number
  int GetBoard() { return board_; }       //!< returns board number
  int GetFEIndex() { return feIndex_; }   //!< returns frontend index
#endif
  std::string GetName();
  int GetDeviceHandle() {
    return device_handle_;                //! returns physical device handle
  }
  HNDLE GetODBHandle() {                  //! returns main ODB handle
    return odb_handle_;
  }
  HNDLE GetSettingsHandle() {             //! returns settings record handle
    return settings_handle_;
  }
  bool GetSettingsTouched() {             //! returns true if odb settings  touched
    return settings_touched_;
  }
  void SetSettingsTouched(bool t) {       //! set _settings_touched
    settings_touched_ = t;
  }
  void SetRingBufferHandle(int rb_handle) { //! set ring buffer index
    rb_handle_ = rb_handle;
  }
  int GetRingBufferHandle() {             //! returns ring buffer index
    return rb_handle_;
  }
  int GetNumEventsInRB() {                //! returns number of events in ring buffer
    return num_events_in_rb_.load();
  }
  DataType GetDataType();
  int GetVerbosity(){
    return verbosity_;
  }
  void SetVerbosity(int verbosity){
    verbosity_ = verbosity;
  }

  /* These are atomic with sequential memory ordering. See below */
  void IncrementNumEventsInRB() {         //! Increment Number of events in ring buffer
    num_events_in_rb_++;
  }
  void DecrementNumEventsInRB() {         //! Decrement Number of events in ring buffer
    num_events_in_rb_--;
  }
  void ResetNumEventsInRB() {             //! Reset Number of events in ring buffer
    num_events_in_rb_=0;
  }

private:

  /* Private fields */

  /* IMPORTANT
   *
   * If adding additional fields, do NOT forget to change the move constructor
   * and move assignment operator accordingly
   */
#if SIMULATION
  int board_,
  moduleID_;
#else
  int feIndex_,           //!< Frontend index number
  link_,                  //!< Optical link number
  board_,                 //!< Module/Board number
  moduleID_;              //!< Unique module ID
#endif

  int device_handle_;     //!< physical device handle
  HNDLE odb_handle_;      //!< main ODB handle
  HNDLE settings_handle_; //!< Handle for the device settings record
  int rb_handle_;         //!< Handle to ring buffer
  bool settings_loaded_;  //!< ODB settings loaded
  bool settings_touched_; //!< ODB settings touched
  bool running_;          //!< Run in progress
  DataType data_type_;    //!< Data type for all channels:
  int verbosity_;         //!< Make the driver verbose
                          //!< 0: off
                          //!< 1: normal
                          //!< 2: very verbose
  /* We use an atomic types here to get lock-free (no pthread mutex lock or spinlock)
   * read-modify-write. operator++(int) and operator++() on an atomic<integral> use
   * atomic::fetch_add() and operator--(int) and operator--() use atomic::fetch_sub().
   * The default memory ordering for these functions is memory_order_seq_cst (sequentially
   * consistent). This saves us from inserting a memory barrier between read/write pointer
   * incrementation and an increment/decrement of this variable.   */
  std::atomic<int> num_events_in_rb_;  //!< Number of events stored in ring buffer

  //! Minimum sample value for each channel. Only used for very temporary storage between
  //  calculating the minima and actually writing them to the ring buffers.
  int channel_minima_[32][8];

  PMT*      pmts[8];  ///< "PMT" object for each channel
  PMT*      pmt;      ///< Current PMT being filled.
  Pulse*    pulse;    ///< Current pulse being filled;

  Float_t                 DefaultBaseline;             ///< Default baseline.
  Float_t                 CurrentDefaultBaseline;      ///< Current default baseline, set to Default or the last well measued baseline for this channel.
  std::vector<UShort_t>   samples;  ///< Samples to do pulse-finding on.
  std::vector<Int_t>      Deriv;    ///< Derivative between samples.
  std::vector<Int_t>      AbsDeriv; ///< Absolute derivative between samples.
  bool saturationFail; ///< Whether some pulses reached 0 ADC
  bool baselineFail;   ///< Whether we failed to calculate the baseline for some pulses
  bool IsAdjacent;

  struct PulseState {
    UInt_t Charge;     ///< Charge of the pulse, relative to 4906.
    UShort_t ChargePeak; ///< Charge up to and including the pulse's peak.
    Float_t  ChargeConv; ///< Charge of pulse.
    UShort_t Start;      ///< Start of the pulse, in bins since start of block.
    UShort_t Stop;       ///< End of the pulse, in bins since start of block.
    UShort_t Offset;     ///< The block offset, in bins since start of block.
    Byte_t   Conf;       ///< How SPE-like the pulse is.
    UShort_t MaxV;       ///< Minimum sample, in ADC.
    UShort_t MinV;       ///< Maximum sample between current peaks in ADCs.
    UShort_t MinVT;      ///< Time of Maximum.
    Int_t    MaxD;       ///< Maximum derivative of a pulse.
    std::vector<UShort_t> MaxVT;      ///< The bin number of each subpeak
    UShort_t GlobalMaxV;
    UShort_t GlobalMaxVT;
    UShort_t MaxVindex;  ///< MaxVT vector index of largest peak.
    UShort_t MaxDindex;  ///< MaxDT vector index.
    Float_t  Charge_MaxD; ///< Charge/MaxD.
    Float_t  Charge_MaxV; ///< Charge/MaxX.
    Float_t  Charge_Frac; ///< ChargePeak/(Charge-ChargePeak).
    Float_t  GlobalMaxD;  ///< Maximum derivative over whole pulse.
    Float_t  BaseV;       ///< Baseline of pulse, in ADC.
    Short_t  BaselineInt;       ///< Sum of baseline samples, relative to 3892.
    Short_t  BaselineSquareInt; ///< Sum of squares of baseline samples, relative to 3892*3892.
    Byte_t   BaselineSamples;   ///< Number of samples used for baseline calc.
    Bool_t   IsLowGain;         ///< Whether the pulse is from a LAr low-gain channel.
    Bool_t   IsV1740;           ///< Whether the pulse is from a V1740 board.
    Bool_t   CanSplit;    ///< Can pulse be split.
    Bool_t   IsSplit;     ///< Indicates if pulse is the tail of a split pulse.
    std::vector<UShort_t> SubnsSamples;
  } pulseState;

  struct ChannelState {
    Int_t   BaselineInt;       ///< Sum of baseline samples, relative to 3892.
    Int_t   BaselineSquareInt; ///< Sum of squares of baseline samples, relative to 3892*3892.
    UShort_t BaselineSamples;  ///< Number of samples used for baseline calc.
    bool    Saturated;
    Float_t TimeOffset; /// Time offset of the channel, in ns.

    /// Whether a raw block has been combined with the previous block and
    /// should be ignored.
    std::vector<Bool_t> IsBlockCombinedWithPrevious;
    /// The offsets of ll the blocks we've analyzed so far.
    std::vector<UShort_t> BlockOffsetsAnalyzed;
  } channelState;

  struct PulseFindingConfig {
    Float_t ChargeCut;              ///< Charge cut for V1720 pulses, in ADC.
    Int_t   SubPeakDerivThreshold;  ///< Threshold for secondary peaks.
    Int_t   DerivThreshold;         ///< Derivative threshold for starting a pulse.
    Int_t   StopDeriv;              ///< Derivative threshold for stopping a pulse.
    Int_t   StopVolt;               ///< Voltage threshold for stopping a pulse.
    Int_t   MinBaselineSamples;     ///< If don't have enough samples, pulse will be PRUNED!
    Int_t   MaxBaselineSamples;     ///< Max # baseline samples. MUST be < 256 or Pulse class won't handle it cleanly.
    Bool_t  DoHiddenPeakFind;       ///< Whether to do extra peak finding in sub-ns function.
    Float_t ChargeResidualLimit;    ///< ChargeResidual cut for doing higher order Sub-ns and peak finding.
    Int_t   ClosePulseHeightThresh; ///< Require lookahead pulses to be below this to close.
    Int_t   ClosePulseDerivThresh;  ///< Require lookahead pulses to be below this to close.
    Int_t   ClosePulseLookAhead;          ///< See FindPulses() documentation.
    Int_t   ClosePulseNearEndOfBlock;     ///< See FindPulses() documentation.
    UShort_t   SplitPulseCharge;          ///< The Charge that decides if a pulse can be split or not.
    Float_t SubPeakChargeCutOff;          ///< Above this charge peaks are not fit for their charge, just integrated.
    Float_t ChargeResidualFracLimit;      ///< Above this limit the brute force fitter is used to find hidden peaks.
    UShort_t SubPeakCountLimit;           ///< Above this limit peaks charges are not fit just integrated.
    UShort_t MaxPasses;                   ///< Maximum number of passes that the brute force fitter can take.
    Bool_t  ExtendPulseEnd;               ///< See FindPulses() documentation.
    Bool_t  EnableSubPeaks;               ///< See FindPulses() documentation.
    Bool_t  EnablePulseSplitting;         ///< See FindPulses() documentation.
    Int_t   MaxSPEWidthIntercept;   /// Max width = (charge + intercept) * slope
    Float_t MaxSPEWidthSlope;       /// Max width = (charge + intercept) * slope
    Int_t   MaxSPECharge;           /// Max charge of SPE pulse in ADC
  } pfConfig;

  struct SPEPDF {
    Int_t   MaxVbins;       ///< Number of MaxV bins in the PDFs.
    Float_t MaxVmin;        ///< Lower limit for MaxV in the PDFs.
    Float_t MaxVmax;        ///< Upper limit for MaxV in the PDFs.
    Int_t   MaxDbins;       ///< Number of MaxD binds in the PDFs.
    Float_t MaxDmin;        ///< Lower limit for MaxD in the PDFs.
    Float_t MaxDmax;        ///< Upper limit for MaxD in the PDFs.
    Int_t   ChargeFracbins; ///< Number of chargefrac bins in the PDFs.
    Float_t ChargeFracmin;  ///< Lower limit for chargefrac in the PDFs.
    Float_t ChargeFracmax;  ///< Upper limit for chargefrac in the PDFs.
    std::vector<Float_t> MaxV_ChargeFrac; ///< MaxV_ChargeFrac PDFs
    std::vector<Float_t> MaxV_MaxD;       ///< MaxV_MaxD PDFs.
  } spePdf;


  Bool_t kFALSE = false;

    /* Private methods */
  CAENComm_ErrorCode AcqCtl_(uint32_t);
  CAENComm_ErrorCode SetupPreset_(int);
  CAENComm_ErrorCode WriteChannelConfig_(uint32_t);
  CAENComm_ErrorCode ReadReg_(DWORD, DWORD*);
  CAENComm_ErrorCode WriteReg_(DWORD, DWORD);
};

#endif // V1720_HXX_INCLUDE

