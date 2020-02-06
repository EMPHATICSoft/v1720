/*****************************************************************************/
/**
\file feoV1720.cxx

\mainpage

\section contents Contents

Standard Midas Frontend for Optical access to the CAEN v1720 using the A3818 CONET2 driver

\subsection organization File organization

- Compile time parameters setting
- MIDAS global variable defintion
- MIDAS function declaration
- Equipment variables
- functions

\subsection routines Callback routines for system transitions

These routines are called whenever a system transition like start/
stop of a run occurs. The routines are called on the following
occations:

- frontend_init:  When the frontend program is started. This routine
                should initialize the hardware.

- frontend_exit:  When the frontend program is shut down. Can be used
                to releas any locked resources like memory, commu-
                nications ports etc.

- begin_of_run:   When a new run is started. Clear scalers, open
                rungates, etc.

- end_of_run:     Called on a request to stop a run. Can send
                end-of-run event and close run gates.

- pause_run:      When a run is paused. Should disable trigger events.

- resume_run:     When a run is resumed. Should enable trigger events.

\subsection notes Notes about this frontend

This frontend has been designed so that it should compile and work
by default without actual actual access to v1720 hardware. We have turned
off portions of code which make use of the driver to the actual hardware.
Where data acquisition should be performed, we generate random data instead
(see v1720CONET2::ReadEvent()). See usage below to use real hardware.

The simulation code assumes the following setup:
- 1 frontend only
- Arbitrary number of v1720 modules
- Event builder not used

The code to use real hardware assumes this setup:
- 1 A3818 PCI-e board per PC to receive optical connections
- NBLINKSPERA3818 links per A3818 board
- NBLINKSPERFE optical links controlled by each frontend
- NB1720PERLINK v1720 modules per optical link (daisy chained)
- NBV1720TOTAL v1720 modules in total
- The event builder mechanism is used

\subsection usage Usage

\subsubsection simulation Simulation
Simply set the Nv1720 macro below to the number of boards you wish to simulate,
compile and run:
    make SIMULATION=1
    ./feoV1720.exe

\subsubsection real Real hardware
Adjust NBLINKSPERA3818, NBLINKSPERFE, NB1720PERLINK and NBV1720TOTAL below according
to your setup.  NBV1720TOTAL / (NBLINKSPERFE * NB1720PERLINK) frontends
must be started in total. When a frontend is started, it must be assigned an index
number:

    ./frontend -i 0

If no index number is supplied, it is assumed that only 1 frontend is used to control
all boards on all links on that computer.

For example, consider the following setup:

    NBLINKSPERA3818    4     // Number of optical links used per A3818
    NBLINKSPERFE       1     // Number of optical links controlled by each frontend
    NB1720PERLINK      2     // Number of daisy-chained v1720s per optical link
    NBV1720TOTAL       32    // Number of v1720 boards in total

We will need 32/(2*2) = 8 frontends (8 indexes; from 0 to 7).  Each frontend
controls 2*2 = 4 v1720 boards.  Compile and run:

    make SIMULATION=0
    ./feoV1720.exe


\section deap DEAP-3600 notes

MIDAS_SERVER_HOST should be set to deap00:7071. Otherwise frontends will require the
option -h deap00:7071

Each frontend will only access one link. To access all boards they
should be run four times on each computer with each of the four indexes.

\subsection deapusage Usage

- on deap01: ./bin/feoV1720.exe -i [ 0, 1, 2, 3 ]
- on deap02: ./bin/feoV1720.exe -i [ 4, 5, 6, 7 ]
- on deap03: ./bin/feoV1720.exe -i [ 8, 9, 10, 11 ]
- on deap04: ./bin/feoV1720.exe -i [ 12, 13, 14, 15 ]

\subsection deapfiles Files

- feoV1720.cxx : Main front-end user code
- v1720CONET2.hxx / v1720CONET2.cxx : Driver class for the v1720 module
  using the CAEN CONET2 (optical) interface


$Id: feov1720.cxx 128 2011-05-12 06:26:34Z alex $
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sched.h>
#include <sys/resource.h>

#include <fstream>
#include <string>
#include <vector>
#include <memory>

#include "midas.h"
#include "v1720CONET2.hxx"

// __________________________________________________________________
// --- General feov1720 parameters


#ifndef NBLINKSPERA3818
#define NBLINKSPERA3818   4   //!< Number of optical links used per A3818
#define NBLINKSPERFE      4   //!< Number of optical links controlled by each frontend
#define NB1720PERLINK     2   //!< Number of daisy-chained v1720s per optical link
#define NBV1720TOTAL      32  //!< Number of v1720 boards in total
#define NBCORES           8   //!< Number of cpu cores, for process/thread locking
#endif

#ifndef HWLOGDIR
#define HWLOGDIR "/home/deap/pro/FrontEnd/v1720"
#endif

#define SLEEP_TIME_BETWEEN_CONNECTS 50 // in milliseconds

#define  EQ_EVID   1                //!< Event ID
#define  EQ_TRGMSK 0                //!< Trigger mask (overwritten in code)
                                    //!< based on feIndex (see _init) 
#define  FE_NAME   "feov1720MTI"       //!< Frontend name

#define UNUSED(x) ((void)(x)) //!< Suppress compiler warnings
//#define DEBUGTHREAD
//#define SYNCEVENTS
const bool SYNCEVENTS_DEBUG = true;

// __________________________________________________________________
// --- MIDAS global variables
extern HNDLE hDB;   //!< main ODB handle
//extern BOOL debug;  //!< debug printouts

/* make frontend functions callable from the C framework */
#ifdef __cplusplus
extern "C" {
#endif

/*-- Globals -------------------------------------------------------*/

//! The frontend name (client name) as seen by other MIDAS clients
char *frontend_name = (char*)FE_NAME;
//! The frontend file name, don't change it
char *frontend_file_name = (char*)__FILE__;
//! frontend_loop is called periodically if this variable is TRUE
BOOL frontend_call_loop = FALSE;
//! a frontend status page is displayed with this frequency in ms
INT display_period = 000;
//! maximum event size produced by this frontend
INT max_event_size = 32 * 222800;
//! maximum event size for fragmented events (EQ_FRAGMENTED)
INT max_event_size_frag = 5 * 1024 * 1024;
//! buffer size to hold events
INT event_buffer_size = 30 * max_event_size + 10000;
//! log of hardware status
std::ofstream hwlog;
std::string hwlog_filename;

bool runInProgress = false; //!< run is in progress
uint32_t timestamp_offset[NBLINKSPERFE*NB1720PERLINK]; //!< trigger time stamp offsets

// __________________________________________________________________
/*-- MIDAS Function declarations -----------------------------------------*/
INT frontend_init();
INT frontend_exit();
INT begin_of_run(INT run_number, char *error);
INT end_of_run(INT run_number, char *error);
INT pause_run(INT run_number, char *error);
INT resume_run(INT run_number, char *error);
INT frontend_loop();
extern void interrupt_routine(void);  //!< Interrupt Service Routine

INT read_event_from_ring_bufs(char *pevent, INT off);
INT read_buffer_level(char *pevent, INT off);
void * link_thread(void *);

// __________________________________________________________________
/*-- Equipment list ------------------------------------------------*/
#undef USE_INT
//! Main structure for midas equipment
EQUIPMENT equipment[] =
{
    {
        "FEV1720MTI%02d",           /* equipment name */
        {
            EQ_EVID, EQ_TRGMSK,     /* event ID, trigger mask */
#if USE_SYSTEM_BUFFER
            "SYSTEM",               /* write events to system buffer */
#else
            "BUF%02d",              /* make different frontends (indexes) write to different buffers */
#endif //USE_SYSTEM_BUFFER
#ifdef USE_INT
            EQ_INTERRUPT,           /* equipment type */
#else
            EQ_POLLED | EQ_EB,      /* equipment type */
#endif //USE_INT

            LAM_SOURCE(0, 0x0),     /* event source crate 0, all stations */
            "MIDAS",                /* format */
            TRUE,                   /* enabled */
            RO_RUNNING,             /* read only when running */
            500,                    /* poll for 500ms */
            0,                      /* stop run after this event limit */
            0,                      /* number of sub events */
            0,                      /* don't log history */
            "", "", ""
        },
        read_event_from_ring_bufs,  /* readout routine */
    },

    {
        "BUFLVLMT%02d",             /* equipment name */
        {
            100, 0x1000,            /* event ID, corrected with feIndex, trigger mask */
            "SYSTEM",               /* event buffer */
            EQ_PERIODIC,            /* equipment type */
            0,                      /* event source */
            "MIDAS",                /* format */
            TRUE,                   /* enabled */
            RO_RUNNING | RO_TRANSITIONS |   /* read when running and on transitions */
            RO_ODB,                 /* and update ODB */
            1000,                   /* read every 1 sec */
            0,                      /* stop run after this event limit */
            0,                      /* number of sub events */
            1,                      /* log history */
            "", "", ""
        },
        read_buffer_level,       /* readout routine */
    },

    {""}
};

#ifdef __cplusplus
}
#endif

std::vector<v1720CONET2> ov1720; //!< objects for the v1720 modules controlled by this frontend
std::vector<v1720CONET2>::iterator itv1720;  //!< Main thread iterator
std::vector<v1720CONET2>::iterator itv1720_thread[NBLINKSPERFE];  //!< Link threads iterators

pthread_t tid[NBLINKSPERFE];                            //!< Thread ID
int thread_retval[NBLINKSPERFE] = {0};                  //!< Thread return value
int thread_link[NBLINKSPERFE];                          //!< Link number associated with each thread

/********************************************************************/
/********************************************************************/
/********************************************************************/
/**
 * \brief   Sequencer callback info
 *
 * Function which gets called when record is updated
 *
 * \param   [in]  h main ODB handle
 * \param   [in]  hseq Handle for record that was updated
 * \param   [in]  info Record descriptor additional info
 */
void seq_callback(INT h, INT hseq, void *info){
  KEY key;

  for (itv1720 = ov1720.begin(); itv1720 != ov1720.end(); ++itv1720) {
    if (hseq == itv1720->GetSettingsHandle()){
      db_get_key(h, hseq, &key);
      itv1720->SetSettingsTouched(true);
      cm_msg(MINFO, "seq_callback", "Settings %s touched. Changes will take effect at start of next run.", key.name);
    }
  }
}

//
//-------------------------------------------------------------------
/**
 * \brief   Frontend initialization
 *
 * Runs once at application startup.  We initialize the hardware and optical
 * interfaces and set the equipment status in ODB.  We also lock the frontend
 *  to once physical cpu core.
 *
 * \return  Midas status code
 */
INT frontend_init(){

  std::stringstream * ss = new std::stringstream();
  int feIndex = get_frontend_index();
  *ss << HWLOGDIR << "/hwlog" << ((feIndex == -1) ? 0 : feIndex);
  hwlog_filename = ss->str();

  set_equipment_status(equipment[0].name, "Initializing...", "#FFFF00");
  printf("<<< Begin of Init\n");

  {
    // Reset the PLL lock loss flag in ODB
    char Path[255];
    sprintf(Path,"/DEAP Alarm/PLL Loss FE0%d",get_frontend_index());
    INT dummy;
    int size=sizeof(INT);
    db_get_value(hDB, 0, Path, &(dummy), &size, TID_INT, true);
    dummy=-1;
    db_set_value(hDB, 0, Path, &(dummy), sizeof(INT), 1, TID_INT);
  }
  {
    // Correct the Trigger mask based on the frontend index, update ODB
    // Used for sorting the threads, and for logger filtering
    char sEpath[64];
    equipment[0].info.trigger_mask = (2<<feIndex);
    snprintf(sEpath, sizeof(sEpath), "Equipment/%s/Common/Trigger mask", equipment[0].name);
    db_set_value(hDB, 0, sEpath, &(equipment[0].info.trigger_mask), sizeof(WORD), 1, TID_WORD);
    
    // Correct the Buffer level equipment Event ID based on the frontend index, update ODB
    equipment[1].info.event_id += feIndex;
    snprintf(sEpath, sizeof(sEpath), "Equipment/%s/Common/Event ID", equipment[1].name);
    db_set_value(hDB, 0, sEpath, &(equipment[1].info.event_id), sizeof(WORD), 1, TID_WORD);
  }

  // --- Suppress watchdog for PICe for now
  cm_set_watchdog_params(FALSE, 0);

  int nExpected = 0; //Number of v1720 boards we expect to activate
  int nActive = 0;   //Number of v1720 boards activated at the end of frontend_init
  std::vector<std::pair<int,int> > errBoards;  //v1720 boards which we couldn't connect to
  
  // If no index was supplied on the command-line, assume 1 frontend
  // to control all the links and boards
  if(feIndex == -1) {
    nExpected = NB1720PERLINK*NBLINKSPERA3818;
    
    printf("<<< No index supplied! Assuming only 1 frontend only and starting all boards on all links\n");
    for (int iLink=0; iLink < NBLINKSPERA3818; iLink++) {
      for (int iBoard=0; iBoard < NB1720PERLINK; iBoard++) {
        printf("==== Link:%d, Board:%d ====\n", iLink, iBoard);
        
        // Compose unique module ID
        int moduleID = iLink*NB1720PERLINK + iBoard;
        
        // Create module objects
        ov1720.emplace_back(feIndex, iLink, iBoard, moduleID, hDB);
        
        ov1720.back().SetVerbosity(0);
        
        // Open Optical interface
        printf("Opening optical interface Link %d, Board %d\n", iLink, iBoard);
        switch(ov1720.back().Connect()){
        case v1720CONET2::ConnectSuccess:
          nActive++;
          break;
        case v1720CONET2::ConnectErrorCaenComm:
        case v1720CONET2::ConnectErrorTimeout:
          errBoards.push_back(std::pair<int,int>(iLink,iBoard));
          break;
        case v1720CONET2::ConnectErrorAlreadyConnected:
          //do nothing
          break;
        default:
          //Can't happen
          break;
        }
        
        if(!((iLink == (NBLINKSPERA3818-1)) && (iBoard == (NB1720PERLINK-1)))){
          printf("Sleeping for %d milliseconds before next board\n", SLEEP_TIME_BETWEEN_CONNECTS);
          ss_sleep(SLEEP_TIME_BETWEEN_CONNECTS);
        }
      }
    }
  } else {  //index supplied
    
    nExpected = NB1720PERLINK*NBLINKSPERFE;
    
    if((NBV1720TOTAL % (NB1720PERLINK*NBLINKSPERFE)) != 0){
      printf("Incorrect setup: the number of boards controlled by each frontend"
             " is not a fraction of the total number of boards.");
    }
    
    int maxIndex = (NBV1720TOTAL/NB1720PERLINK)/NBLINKSPERFE - 1;
    if(feIndex < 0 || feIndex > maxIndex){
      printf("Front end index must be between 0 and %d\n", maxIndex);
      exit(FE_ERR_HW);
    }
    
    int firstLink = (feIndex % (NBLINKSPERA3818 / NBLINKSPERFE)) * NBLINKSPERFE;
    int lastLink = firstLink + NBLINKSPERFE - 1;
    for (int iLink=firstLink; iLink <= lastLink; iLink++) {
      for (int iBoard=0; iBoard < NB1720PERLINK; iBoard++) {
        printf("==== feIndex:%d, Link:%d, Board:%d ====\n", feIndex, iLink, iBoard);
        
        // Compose unique module ID
        int moduleID = feIndex*NBLINKSPERFE*NB1720PERLINK + (iLink-firstLink)*NB1720PERLINK + iBoard;
        
        // Create module objects
        ov1720.emplace_back(feIndex, iLink, iBoard, moduleID, hDB);
        ov1720.back().SetVerbosity(0);
        
        // Open Optical interface
        switch(ov1720.back().Connect()){
        case v1720CONET2::ConnectSuccess:
          nActive++;
          break;
        case v1720CONET2::ConnectErrorCaenComm:
        case v1720CONET2::ConnectErrorTimeout:
          errBoards.push_back(std::pair<int,int>(iLink,iBoard));
          break;
        case v1720CONET2::ConnectErrorAlreadyConnected:
          //do nothing
          break;
        default:
          //Can't happen
          break;
        }

        if(!((iLink == lastLink) && (iBoard == (NB1720PERLINK-1)))){
          printf("Sleeping for %d milliseconds before next board\n", SLEEP_TIME_BETWEEN_CONNECTS);
          ss_sleep(SLEEP_TIME_BETWEEN_CONNECTS);
        }
      }
    }
  }

  /* This must be done _after_ filling the vector because we pass a pointer to config
   * to db_open_record.  The location of the object in memory must not change after
   * doing that. */
  int nInitOk = 0;
  for (itv1720 = ov1720.begin(); itv1720 != ov1720.end(); ++itv1720) {
    if (! itv1720->IsConnected()) continue;   // Skip unconnected board
    
    // Setup ODB record (create if necessary)
    itv1720->SetBoardRecord(hDB,seq_callback);
    // Set history ODB record (create if necessary)
    itv1720->SetHistoryRecord(hDB,seq_callback);

    int status = itv1720->InitializeForAcq();
    nInitOk += status;
  }
  
  // Abort if board status not Ok.
  if (nInitOk != 0) return FE_ERR_HW;
  
  printf(">>> End of Init. %d active v1720. Expected %d\n\n", nActive, nExpected);

  if(nActive == nExpected){
    set_equipment_status(equipment[0].name, "Initialized", "#00ff00");
  }
  else{
    return FE_ERR_HW;
  }

  cpu_set_t mask;
  CPU_ZERO(&mask);
  CPU_SET(0, &mask);  //Main thread to core 0
  if( sched_setaffinity(0, sizeof(mask), &mask) < 0 ){
    printf("ERROR setting cpu affinity for main thread: %s\n", strerror(errno));
  }

  return SUCCESS;
}

//
//----------------------------------------------------------------------------
/**
 * \brief   Frontend exit
 *
 * Runs at frontend shutdown.  Disconnect hardware and set equipment status in ODB
 *
 * \return  Midas status code
 */
INT frontend_exit(){

  set_equipment_status(equipment[0].name, "Exiting...", "#FFFF00");

  for (itv1720 = ov1720.begin(); itv1720 != ov1720.end(); ++itv1720) {
    if (itv1720->IsConnected()){
      itv1720->Disconnect();
    }
  }
  set_equipment_status(equipment[0].name, "Exited", "#00ff00");
  return SUCCESS;
}

/**
 * \brief   Begin of Run
 *
 * Called every run start transition.  Set equipment status in ODB,
 * start acquisition on the modules.
 *
 * \param   [in]  run_number Number of the run being started
 * \param   [out] error Can be used to write a message string to midas.log
 */
INT begin_of_run(INT run_number, char *error){

  hwlog.open(hwlog_filename.c_str(), std::ios::app);
  hwlog << "========================================================= BOR: (RUN #: "<< run_number << " TIME = " << ss_time() << ") ===================================================" << std::endl;
  hwlog.close();

  set_equipment_status(equipment[0].name, "Starting run...", "#FFFF00");
  cm_msg(MINFO,"BOR", "Start of begin_of_run");
  printf("<<< Start of begin_of_run\n");
  
  int rb_handle;
  int status;
  
  runInProgress = true;
  {
    // Reset the PLL lock loss flag in ODB
    char Path[255];
    sprintf(Path,"/DEAP Alarm/PLL Loss FE0%d",get_frontend_index());
    INT dummy = -1;
    db_set_value(hDB, 0, Path, &(dummy), sizeof(INT), 1, TID_INT);
  }

  for (itv1720 = ov1720.begin(); itv1720 != ov1720.end(); ++itv1720) {
    if (! itv1720->IsConnected()) continue;   // Skip unconnected board
    DWORD vmeAcq, vmeStat;
    itv1720->ReadReg(V1720_ACQUISITION_STATUS, &vmeAcq);//Test the PLL lock once (it may have happened earlier)
    if ((vmeAcq & 0x80) == 0) {
      cm_msg(MERROR,"BeginOfRun","V1720 PLL loss lock Board (sometime in the past):%d (vmeAcq=0x%x)"
             ,itv1720->GetModuleID(), vmeAcq);
      // PLL loss lock reset by the VME_STATUS read!
      itv1720->ReadReg(V1720_VME_STATUS, &vmeStat);
      usleep(100);
      itv1720->ReadReg(V1720_ACQUISITION_STATUS, &vmeAcq); // Test the PLL again 
      if ((vmeAcq & 0x80) == 0) {
        cm_msg(MERROR,"BeginOfRun","V1720 PLL lock still lost Board: %d (vmeAcq=0x%x)"
               ,itv1720->GetModuleID(), vmeAcq);
        return FE_ERR_HW;
      }
    }
    
    // Done in frontend_init, or StartRun if settings have changed
    // itv1720->InitializeForAcq(); 

    bool go = itv1720->StartRun();
    if (go == false) return FE_ERR_HW;
    
    //Create ring buffer for board
    status = rb_create(event_buffer_size, max_event_size, &rb_handle);
    if(status == DB_SUCCESS){
      itv1720->SetRingBufferHandle(rb_handle);
    }
    else{
      cm_msg(MERROR, "feov1720:BOR", "Failed to create rb for board %d", itv1720->GetModuleID());
    }
  }
  
  // Create one thread per optical link
  for(int i=0; i<NBLINKSPERFE; ++i){
    thread_link[i] = i;
    status = pthread_create(&tid[i], NULL, &link_thread, (void*)&thread_link[i]);
    if(status){
      cm_msg(MERROR,"feov1720:BOR", "Couldn't create thread for link %d. Return code: %d", i, status);
    }
  }
  
  set_equipment_status(equipment[0].name, "Started run", "#00ff00");
  printf(">>> End of begin_of_run\n\n");
  
  return SUCCESS;
}

//
//----------------------------------------------------------------------------
void * link_thread(void * arg)
{
  int link = *(int*)arg;
  std::cout << "Started thread for link " << link << " out of " << NBCORES << " cores" << std::endl;

  //Lock each thread to a different cpu core
  cpu_set_t mask;
  CPU_ZERO(&mask);
  switch(NBCORES){
  case 1:
    //Don't do anything
    break;
  case 2:
    CPU_SET(link % 2, &mask); //TRIUMF test PC. Even boards on core 0, odd boards on core 1
    break;
  default:
    /* This will spread the threads on all cores except core 0 when the main thread resides.
     * ex 1 (SNOLAB): NBCORES=8, 4 threads:
     * threads (links) 0,1,2,3 will go on cores 1,2,3,4
     * ex 2: NBCORES 4, 4 threads:
     * threads (links) 0,1,2,3 will go on cores 1,2,3,1     */
    CPU_SET((link % (NBCORES-1)) + 1, &mask);
    printf("core setting: NBCORES:%d link:%d\n", NBCORES, link);
    break;
  }
  if( sched_setaffinity(0, sizeof(mask), &mask) < 0 ){
    printf("ERROR setting cpu affinity for thread %d: %s\n", link, strerror(errno));
  }

  void *wp;
  int status;
  int rb_handle;
  int moduleID;
  int rb_level;
  int firstBoard = link*NB1720PERLINK; //First board on this link

  while(1) {  // Indefinite until run stopped (!runInProgress)
    // This loop is running until the DTM has released the EOR flag (runInProgress)
    // We expect that by that time, the HW buffers (eStored) will be empty
    // If the EOR is set while B02 is processed, there is a possibility that B00..02 
    // will still have data in the HW buffer but looked at as the break is happening!
    // In principle once the EOR is asserted, the for loop should run once more time.
    // Not done in here.
    // process the addressed board for that link only
    for (itv1720_thread[link] = ov1720.begin() + firstBoard;
         itv1720_thread[link] != ov1720.begin() + firstBoard + NB1720PERLINK;
         ++itv1720_thread[link]){

      // Shortcut
      rb_handle = itv1720_thread[link]->GetRingBufferHandle();
      moduleID = itv1720_thread[link]->GetModuleID();

      // Check if event in hardware to read
      if (itv1720_thread[link]->CheckEvent()){
        /* If we've reached 75% of the ring buffer space, don't read
         * the next event.  Wait until the ring buffer level goes down.
         * It is better to let the v1720 buffer fill up instead of
         * the ring buffer, as this the v1720 will generate the HW busy to the 
         * DTM.
         */
        rb_get_buffer_level(rb_handle, &rb_level);
        if(rb_level > (int)(event_buffer_size*0.75)) {
          continue;
        }
        
        // Ok to read data
        status = rb_get_wp(rb_handle, &wp, 100);
        if (status == DB_TIMEOUT) {
          cm_msg(MERROR,"link_thread", "Got wp timeout for thread %d (module %d).  Is the ring buffer full?",
                 link, moduleID);
          cm_msg(MERROR,"link_thread", "Exiting thread %d", link);
          thread_retval[link] = -1;
          pthread_exit((void*)&thread_retval[link]);
        }
        
#ifdef DEBUGTHREAD
        printf("THREAD %d (module %d): Found event\n", link, moduleID);
#endif
        // Read data
        if(itv1720_thread[link]->ReadEvent(wp)) {
          
#ifdef DEBUGTHREAD
          printf("THREAD %d (module %d): Successfully read event\n", link, moduleID);
          printf("THREAD %d (module %d): Events in buffer: %d\n", link, moduleID, itv1720_thread[link]->GetNumEventsInRB());
#endif
        } else {
          cm_msg(MERROR,"link_thread", "Readout routine error on thread %d (module %d)", link, moduleID);
          cm_msg(MERROR,"link_thread", "Exiting thread %d", link);
          thread_retval[link] = -1;
          pthread_exit((void*)&thread_retval[link]);
        }
      } // CheckEvent

      // Sleep for 5us to avoid hammering the board too much
      usleep(5);
    } // Done will all the modules

    // Escape if run is done -> kill thread
    if(!runInProgress)
      break;
  }
  
  cm_msg(MINFO,"link_thread", "Exiting thread %d", link);
  thread_retval[link] = 0;
  pthread_exit((void*)&thread_retval[link]);
}

//
//----------------------------------------------------------------------------
/**
 * \brief   End of Run
 *
 * Called every stop run transition. Set equipment status in ODB,
 * stop acquisition on the modules.
 *
 * \param   [in]  run_number Number of the run being ended
 * \param   [out] error Can be used to write a message string to midas.log
 */
INT end_of_run(INT run_number, char *error)
{
  hwlog.open(hwlog_filename.c_str(), std::ios::app);
  hwlog << "========================================================= EOR: (RUN #: "<< run_number << " TIME = " << ss_time() << ") ===================================================" << std::endl;
  hwlog.close();

  set_equipment_status(equipment[0].name, "Ending run...", "#FFFF00");
  cm_msg(MINFO,"EOR", "Start of end_of_run");
  printf("<<< Start of end_of_run \n");

  DWORD eStored;
  bool result;
  int * status;

  if(runInProgress){  //skip actions if we weren't running

    runInProgress = false;  //Signal threads to quit

    // Do not quite parent before children processes, wait for the proper
    // child exit first.
    for(int i=0; i < NBLINKSPERFE; ++i){
      pthread_join(tid[i],(void**)&status);
      printf(">>> Thread %d joined, return code: %d\n", i, *status);
    }

    // Stop run
    for (itv1720 = ov1720.begin(); itv1720 != ov1720.end(); ++itv1720) {
      if (itv1720->IsConnected()) {  // Skip unconnected board
        result = itv1720->StopRun();

        if(!result)
          cm_msg(MERROR, "EOR",
                 "Could not stop the run for module %d", itv1720->GetModuleID());

        rb_delete(itv1720->GetRingBufferHandle());
        itv1720->SetRingBufferHandle(-1);
		itv1720->ResetNumEventsInRB();
      }
    }

    // Info about event in HW buffer
    result = ov1720[0].Poll(&eStored);
    if(eStored != 0x0) {
      cm_msg(MERROR, "EOR", "Events left in the v1720: %d",eStored);
    }

  }

  printf(">>> End Of end_of_run\n\n");
  set_equipment_status(equipment[0].name, "Ended run", "#00ff00");

  return SUCCESS;
}

//
//----------------------------------------------------------------------------
/**
 * \brief   Pause Run
 *
 * Called every pause run transition.
 *
 * \param   [in]  run_number Number of the run being ended
 * \param   [out] error Can be used to write a message string to midas.log
 *
 * \return  Midas status code
 */
INT pause_run(INT run_number, char *error)
{
  hwlog.open(hwlog_filename.c_str(), std::ios::app);
  hwlog << "========================================================= PAUSE: (RUN #: "<< run_number << " TIME = " << ss_time() << ") =================================================" << std::endl;
  hwlog.close();

  cm_msg(MINFO,"PAUSE", "Beginning of pause_run");
  printf("<<< Beginning of pause_run \n");

  DWORD eStored;
  bool result;
  int * status;

  if(runInProgress){  //skip actions if we weren't running

    runInProgress = false;  //Signal threads to quit

    for(int i=0; i < NBLINKSPERFE; ++i){
      pthread_join(tid[i],(void**)&status);
      printf(">>> Thread %d joined, return code: %d\n", i, *status);
    }

    // Stop run
    for (itv1720 = ov1720.begin(); itv1720 != ov1720.end(); ++itv1720) {
      if (itv1720->IsConnected()) {  // Skip unconnected board
        result = itv1720->StopRun();

        if(!result)
          cm_msg(MERROR, "EOR",
                 "Could not stop the run for module %d", itv1720->GetModuleID());

        rb_delete(itv1720->GetRingBufferHandle());
        itv1720->SetRingBufferHandle(-1);
		itv1720->ResetNumEventsInRB();
      }
    }

    result = ov1720[0].Poll(&eStored);
    if(eStored != 0x0) {
      cm_msg(MERROR, "EOR", "Events left in the v1720: %d",eStored);
    }

  }

  cm_msg(MINFO,"PAUSE", "End of pause_run");
  printf("<<< End of pause_run \n");
  return SUCCESS;
}

//
//----------------------------------------------------------------------------
/**
 * \brief   Resume Run
 *
 * Called every resume run transition.
 *
 * \param   [in]  run_number Number of the run being ended
 * \param   [out] error Can be used to write a message string to midas.log
 *
 * \return  Midas status code
 */
INT resume_run(INT run_number, char *error)
{

  hwlog.open(hwlog_filename.c_str(), std::ios::app);
  hwlog << "========================================================= RESUME: (RUN #: "<< run_number << " TIME = " << ss_time() << ") ================================================" << std::endl;
  hwlog.close();

  printf("<<< Beginning of resume_run \n");

  int rb_handle;
  int status;

  runInProgress = true;

  for (itv1720 = ov1720.begin(); itv1720 != ov1720.end(); ++itv1720) {
    if (! itv1720->IsConnected()) continue;   // Skip unconnected board

    // Done in frontend_init, or StartRun if settings have changed
    // itv1720->InitializeForAcq(); 

    bool go = itv1720->StartRun();
    if (go == false) return FE_ERR_HW;

    //Create ring buffer for board
    status = rb_create(event_buffer_size, max_event_size, &rb_handle);
    if(status == DB_SUCCESS){
      itv1720->SetRingBufferHandle(rb_handle);
    }
    else{
      cm_msg(MERROR, "feov1720:BOR", "Failed to create rb for board %d", itv1720->GetModuleID());
    }
  }

  //Create one thread per optical link
  for(int i=0; i<NBLINKSPERFE; ++i){
    thread_link[i] = i;
    status = pthread_create(&tid[i], NULL, &link_thread, (void*)&thread_link[i]);
    if(status){
      cm_msg(MERROR,"feov1720:BOR", "Couldn't create thread for link %d. Return code: %d", i, status);
    }
  }

  printf("<<< End of resume_run \n");
  return SUCCESS;
}

//
//----------------------------------------------------------------------------
DWORD prevtime = 0;
INT numloops = 0;
DWORD sn=0;
/**
 * \brief   Frontend loop
 *
 * If frontend_call_loop is true, this routine gets called when
 * the frontend is idle or once between every event.
 *
 * \return  Midas status code
 */
INT frontend_loop()
{
  if((prevtime == 0) || ((ss_time() - prevtime) > 2)){

    hwlog.open(hwlog_filename.c_str(), std::ios::app);

    // Header, print when frontend starts
    if(prevtime == 0){
      hwlog << "=================================================================================================================================================" << std::endl;
      hwlog << "========================================================= FE START: (TIME = " << ss_time() << ") =========================================================" << std::endl;
      hwlog << "=================================================================================================================================================" << std::endl;
    }
    // Subheader, print every 80 lines
    if((numloops % 80) == 0){
      hwlog << "\t   |  " << "Board 0" << "\t\t\t\t\t\t\t\t|  " << "Board 1" << std::endl;
      hwlog << "TIME" << "\t   |  "
            << "VME_STATUS " << "VME_CTL    " << "ACQ_STATUS " << "ACQ_CTL    " << "EV_STORED" << "  SN" << "         |  "
            << "VME_STATUS " << "VME_CTL    " << "ACQ_STATUS " << "ACQ_CTL    " << "EV_STORED" << "  SN" << std::endl;
    }

    //Get previous time, pad it to 11 spaces and write to log file
    prevtime = ss_time();
    std::stringstream sstime;
    sstime << prevtime;
    for(int i=sstime.str().length(); i<11; ++i)
      sstime << " ";
    hwlog << sstime.str() << "|  ";

    for (itv1720 = ov1720.begin(); itv1720 != ov1720.end(); ++itv1720) {
      if (! itv1720->IsConnected()){ // Skip unconnected board
        hwlog << std::endl;
        continue;
      }

      DWORD vme_status, vme_ctl, acq_status, acq_ctl, ev_stored;
      std::vector<std::unique_ptr<std::stringstream> > vss;

      // Query info from hardware
      itv1720->ReadReg(V1720_VME_STATUS, &vme_status);
      vss.push_back(std::unique_ptr<std::stringstream>(new std::stringstream));
      *vss.back() << "0x" << std::hex << vme_status;
      itv1720->ReadReg(V1720_VME_CONTROL, &vme_ctl);
      vss.push_back(std::unique_ptr<std::stringstream>(new std::stringstream));
      *vss.back() << "0x" << std::hex << vme_ctl;
      itv1720->ReadReg(V1720_ACQUISITION_STATUS, &acq_status);
      vss.push_back(std::unique_ptr<std::stringstream>(new std::stringstream));
      *vss.back() << "0x" << std::hex << acq_status;
      itv1720->ReadReg(V1720_ACQUISITION_CONTROL, &acq_ctl);
      vss.push_back(std::unique_ptr<std::stringstream>(new std::stringstream));
      *vss.back() << "0x" << std::hex << acq_ctl;
      itv1720->ReadReg(V1720_EVENT_STORED, &ev_stored);
      vss.push_back(std::unique_ptr<std::stringstream>(new std::stringstream));
      *vss.back() << std::dec << ev_stored;
      vss.push_back(std::unique_ptr<std::stringstream>(new std::stringstream));
      *vss.back() << std::dec << sn;

      // Body
      for(unsigned int i=0; i<vss.size(); ++i){
        // Pad the data with spaces up to 11 characters for pretty display
        for(int j=vss[i]->str().length(); j<11; ++j)
          *vss[i] << " ";
        hwlog << vss[i]->str();
      }
      if ((ov1720.end() - itv1720) == 1)
        hwlog << std::endl;
      else
        hwlog << "|  ";

    }
    numloops++;
  }

  hwlog.close();

  return SUCCESS;
}

/*------------------------------------------------------------------*/
/********************************************************************\
  Readout routines for different events
\********************************************************************/
int Nloop;  //!< Number of loops executed in event polling
int Ncount; //!< Loop count for event polling timeout
DWORD acqStat; //!< ACQUISITION STATUS reg, must be global because read by poll_event, accessed by read_trigger_event
// ___________________________________________________________________
/*-- Trigger event routines ----------------------------------------*/
/**
 * \brief   Polling routine for events.
 *
 * \param   [in]  source Event source (LAM/IRQ)
 * \param   [in]  count Loop count for event polling timeout
 * \param   [in]  test flag used to time the polling
 * \return  1 if event is available, 0 if done polling (no event).
 * If test equals TRUE, don't return.
 */
extern "C" INT poll_event(INT source, INT count, BOOL test)
{

  register int i;

  for (i = 0; i < count; i++) {

    //ready for readout only when data is present in all ring buffers
    bool evtReady = true;
    for (itv1720 = ov1720.begin(); itv1720 != ov1720.end(); ++itv1720){

      //      if(itv1720->GetNumEventsInRB() == 0){
      if(itv1720->IsConnected() && (itv1720->GetNumEventsInRB() == 0)){
        evtReady = false;
      }
    }

    //If event not ready or we're in test phase, keep looping
    if (evtReady && !test)
      //      return 0;
      return 1;

		usleep(1);
  }
  return 0;
}

//
//----------------------------------------------------------------------------
/**
 * \brief   Interrupt configuration (not implemented)
 *
 * Routine for interrupt configuration if equipment is set in EQ_INTERRUPT
 * mode.  Not implemented right now, returns SUCCESS.
 *
 * \param   [in]  cmd Command for interrupt events (see midas.h)
 * \param   [in]  source Equipment index number
 * \param   [in]  adr Interrupt routine (see mfe.c)
 *
 * \return  Midas status code
 */
extern "C" INT interrupt_configure(INT cmd, INT source, POINTER_T adr)
{
  switch (cmd) {
  case CMD_INTERRUPT_ENABLE:
    break;
  case CMD_INTERRUPT_DISABLE:
    break;
  case CMD_INTERRUPT_ATTACH:
    break;
  case CMD_INTERRUPT_DETACH:
    break;
  }
  return SUCCESS;
}

//
//----------------------------------------------------------------------------
/**
 * \brief   Event readout
 *
 * Event readout routine.  This is called by the polling or interrupt routines.
 * (see mfe.c).  For each module, read the event buffer into a midas data bank.
 * If ZLE data exists, create another bank for it.  Finally, create a statistical
 * bank for data throughput analysis.
 *
 * \param   [in]  pevent Pointer to event buffer
 * \param   [in]  off Caller info (unused here), see mfe.c
 *
 * \return  Size of the event
 */
INT read_event_from_ring_bufs(char *pevent, INT off) {

  if (!runInProgress) return 0;

  sn = SERIAL_NUMBER(pevent);

  //#ifdef SYNCEVENTS
#if 0
  /* Check ring buffers to see if the event counters match.
   * If not, discard old events. */
  uint32_t event_header[4];
  uint32_t timestamp[NBLINKSPERFE*NB1720PERLINK];
  uint32_t highest_timestamp;
  uint32_t lowest_timestamp;
  int idx;
  bool empty_buffers = false;

  printf("\n### sn: %u\n", sn);

#if 0 //********* Testing resync ********

  /* This code will produce a situation where when we hit event serial number 4,
   * we have ring buffers with 2 events (boards 0,3), 3 events (boards 5,6,7)
   * and 1 event (boards 1,2,4).  The extra events in boards 0,3,5,6,7 have older
   * timestamps and must be removed to resynchronize the boards and compose
   * the final MIDAS event.
   */
  static int numattempts = 1;
  if(sn == 0){
    numattempts = 1;
  }
  if(sn == 1){
    /* Write one random bank instead of calling
     * FillBufferEvent(), so we keep the events
     * in the ring buffers for now */
    printf("### Filling ring buffers...\n");
    bk_init32(pevent);
    // Write random bank
    DWORD *dest;
    bk_create(pevent, "TEST", TID_DWORD, (void **)&dest);
    *dest = 1;
    bk_close(pevent, dest + 1);

    for (itv1720 = ov1720.begin(); itv1720 != ov1720.end(); ++itv1720) {
      if (! itv1720->IsConnected()) continue;   // Skip unconnected board
      printf("### Module ID: %d, rb after sn 1: %d\n", itv1720->GetModuleID(), itv1720->GetNumEventsInRB());
    }

    INT ev_size = bk_size(pevent);
    if(ev_size == 0)
      cm_msg(MINFO,"read_trigger_event", "******** Event size is 0, SN: %d", sn);
    //  return bk_size(pevent);
    return ev_size;

  }
  if(sn == 2){
    //Desynchronize some boards with respect to the others
    printf("### Desynchronizing...\n");
    bk_init32(pevent);
    for (itv1720 = ov1720.begin(); itv1720 != ov1720.end(); ++itv1720) {
      if (! itv1720->IsConnected()) continue;   // Skip unconnected board
      // >>> Fill Event bank
      int idx = itv1720 - ov1720.begin();
      switch(idx){
      case 0:
      case 3:
      case 5:
      case 6:
      case 7:
        //don't fill bank so don't remove event from ring buffer
        break;
      default:
        itv1720->FillEventBank(pevent);
        break;
      }
    }

    for (itv1720 = ov1720.begin(); itv1720 != ov1720.end(); ++itv1720) {
      if (! itv1720->IsConnected()) continue;   // Skip unconnected board
      printf("### Module ID: %d, rb after sn 2: %d\n", itv1720->GetModuleID(), itv1720->GetNumEventsInRB());
    }

    INT ev_size = bk_size(pevent);
    if(ev_size == 0)
      cm_msg(MINFO,"read_trigger_event", "******** Event size is 0, SN: %d", sn);
    //  return bk_size(pevent);
    return ev_size;
  }
  if(sn == 3){
    //Desynchronize further
    printf("### Desynchronizing further...\n");
    bk_init32(pevent);
    for (itv1720 = ov1720.begin(); itv1720 != ov1720.end(); ++itv1720) {
      if (! itv1720->IsConnected()) continue;   // Skip unconnected board

      // >>> Fill Event bank
      int idx = itv1720 - ov1720.begin();
      switch(idx){
      case 5:
      case 6:
      case 7:
        //don't fill bank so don't remove event from ring buffer
        break;
      default:
        itv1720->FillEventBank(pevent);
        break;
      }
    }

    for (itv1720 = ov1720.begin(); itv1720 != ov1720.end(); ++itv1720) {
      if (! itv1720->IsConnected()) continue;   // Skip unconnected board
      printf("### Module ID: %d, rb after sn 3: %d\n", itv1720->GetModuleID(), itv1720->GetNumEventsInRB());
    }

    INT ev_size = bk_size(pevent);
    if(ev_size == 0)
      cm_msg(MINFO,"read_trigger_event", "******** Event size is 0, SN: %d", sn);
    //  return bk_size(pevent);
    return ev_size;
  }
  if(sn == 4){
    /* Since the polling loop will always return true because there are events
     * in all ring buffers, we need to give the threads some time to fill the
     * event buffers until they are filled up like we want.  Give up after 100
     * attempts and proceed with whatever number of events we have.  */
    if(numattempts < 100){

      if((ov1720[0].GetNumEventsInRB() != 2) ||
         (ov1720[1].GetNumEventsInRB() != 1) ||
         (ov1720[2].GetNumEventsInRB() != 1) ||
         (ov1720[3].GetNumEventsInRB() != 2) ||
         (ov1720[4].GetNumEventsInRB() != 1) ||
         (ov1720[5].GetNumEventsInRB() != 3) ||
         (ov1720[6].GetNumEventsInRB() != 3) ||
         (ov1720[7].GetNumEventsInRB() != 3)){

        printf("### Wrong number of events in buffers (attempt %d), return...\n", numattempts);
        usleep(100);
        ++numattempts;
        return 0;
      }
    }
  }
  //********************************
#endif //********* Testing resync ********

  /* Since the trigger time stamps (TTT) are not perfectly aligned,
   * (+/- 1 clock cycle), memorize the time stamps of the first event
   * to use as offsets for subsequent events.  */
  if(sn == 0) {
    for (itv1720 = ov1720.begin(); itv1720 != ov1720.end(); ++itv1720) {
      if (! itv1720->IsConnected()) continue;   // Skip unconnected board

      itv1720->FetchHeaderNextEvent(event_header);
      timestamp_offset[itv1720 - ov1720.begin()] = event_header[3] & ~(0x80000000);//ignore HSB on timestamp (roll-over bit);
      
      if(SYNCEVENTS_DEBUG) printf("Module: %d, rb: %d, event: %u, TS: 0x%08x\n"
                                  ,iv1720->GetModuleID()
                                  , itv1720->GetNumEventsInRB()
                                  , event_header[2] & 0x00FFFFFF
                                  , event_header[3] & ~(0x80000000));
    }
  } else {
    
    //Loop until all next events in ring buffers match
    while(1) {
      
      /* Check if events match and find highest event counter
       * Correct for timestamp offsets by subtracting the timestamps
       * recorded for event 0 */
      itv1720 = ov1720.begin();
      itv1720->FetchHeaderNextEvent(event_header);
      //ignore HSB on timestamp (roll-over bit)
      timestamp[0] = (event_header[3] & ~(0x80000000)) - timestamp_offset[0];

      highest_timestamp = lowest_timestamp = timestamp[0];

      if(SYNCEVENTS_DEBUG) printf("Module: %d, rb: %d, event: %u, TS: 0x%08x\n",
                                  itv1720->GetModuleID(), itv1720->GetNumEventsInRB(), event_header[2] & 0x00FFFFFF, event_header[3] & ~(0x80000000));
      if(SYNCEVENTS_DEBUG) printf("Module: %d, rb: %d, event: %u, TS - offset: 0x%08x\n",
                                  itv1720->GetModuleID(), itv1720->GetNumEventsInRB(), event_header[2] & 0x00FFFFFF, timestamp[0]);

      ++itv1720;
      idx = itv1720 - ov1720.begin();
      for (; itv1720 != ov1720.end();) {
        if (! itv1720->IsConnected()) continue;   // Skip unconnected board

        itv1720->FetchHeaderNextEvent(event_header);
        timestamp[idx] = (event_header[3] & ~(0x80000000)) - timestamp_offset[idx];

        if(timestamp[idx] > highest_timestamp)
          highest_timestamp = timestamp[idx];
        else if(timestamp[idx] < lowest_timestamp)
          lowest_timestamp = timestamp[idx];

        if(SYNCEVENTS_DEBUG) printf("Module: %d, rb: %d, event: %u, TS: 0x%08x\n",
                                    itv1720->GetModuleID(), itv1720->GetNumEventsInRB(), event_header[2] & 0x00FFFFFF, event_header[3] & ~(0x80000000));
        if(SYNCEVENTS_DEBUG) printf("Module: %d, rb: %d, event: %u, TS - offset: 0x%08x\n",
                                    itv1720->GetModuleID(), itv1720->GetNumEventsInRB(), event_header[2] & 0x00FFFFFF, timestamp[idx]);

        ++itv1720;
        ++idx;
      }

      /* If timestamps differ by more than a few clock cycles, then the events in the
       * ring buffers are not synchronized.  We therefore need to get rid of the older
       * events until we are re-synchronized. We use a window of 96ns (each timestamp
       * count = 8ns -> 12*8ns = 96ns) in which we consider the board events to belong
       * to the same physical event */
      const uint32_t EVENT_TIME_WINDOW = 12;  //in timestamp counts (1 count = 8ns)
      if((highest_timestamp - lowest_timestamp) < EVENT_TIME_WINDOW){
        break;
      }
      else {

        if(SYNCEVENTS_DEBUG) printf("### Events don't match!, removing older events from ring buffers...\n");

        for (itv1720 = ov1720.begin(); itv1720 != ov1720.end(); ++itv1720) {
          if (! itv1720->IsConnected()) continue;   // Skip unconnected board

          idx = itv1720 - ov1720.begin();


          if(SYNCEVENTS_DEBUG) printf("### idx: %d, rb: %d, timestamp[idx]: 0x%08x, highest_timestamp - 12: 0x%08x\n",
                                      idx, itv1720->GetNumEventsInRB(), timestamp[idx], highest_timestamp - 12);

          if(timestamp[idx] <  highest_timestamp - 12){

            /* No need to mutex/spin lock this, since the only
             * other place where the write pointer is incremented
             * is in FillEventBank() called later in this function */
            if(SYNCEVENTS_DEBUG) printf("### Removing next event for module %d\n", idx);
            itv1720->DeleteNextEvent();

            /* If that was the last event in the ring buffer,
             * we can no longer compose the MIDAS event, return
             * and wait for the next trigger     */
            if(itv1720->GetNumEventsInRB() == 0){
              if(SYNCEVENTS_DEBUG) printf("### Ring buffer %d is empty\n", itv1720->GetModuleID());
              empty_buffers = true;
            }
          }
        }

        if(empty_buffers){
          if(SYNCEVENTS_DEBUG) printf("### One or more ring buffers empty!, returning...\n");
          return 0;
        }
        //else keep looping
      }

    } //while(1)
  }

  //#endif //SYNCEVENTS
#endif

	// >>> Get time before read (for data throughput analysis. To be removed)
	//timeval tv, tv1;
	//gettimeofday(&tv,0);
	suseconds_t usStart = 0;//tv.tv_usec;


  bk_init32(pevent);
  for (itv1720 = ov1720.begin(); itv1720 != ov1720.end(); ++itv1720) {
    if (! itv1720->IsConnected()) continue;   // Skip unconnected board


    // >>> Fill Event bank
    itv1720->FillEventBank(pevent);

    // >>> Fill statistical bank
	// itv1720->FillStatBank(pevent, usStart);

  }

  //primitive progress bar
  if (sn % 1000 == 0) printf(".");// printf("%i %i .\n",endStart-usStart,total_event_bank);

  INT ev_size = bk_size(pevent);
  if(ev_size == 0)
    cm_msg(MINFO,"read_trigger_event", "******** Event size is 0, SN: %d", sn);
  //  return bk_size(pevent);
  return ev_size;
}

//                                                                                         
//----------------------------------------------------------------------------             
INT read_buffer_level(char *pevent, INT off) {
  
  bk_init32(pevent);
  int PLLLockLossID = -1;
  for (itv1720 = ov1720.begin(); itv1720 != ov1720.end(); ++itv1720){
    itv1720->FillBufferLevelBank(pevent);
		// Check the PLL lock
    DWORD vmeStat, vmeAcq;
    itv1720->ReadReg(V1720_ACQUISITION_STATUS, &vmeAcq);
    if ((vmeAcq & 0x80) == 0) {
      PLLLockLossID= itv1720->GetModuleID();
      cm_msg(MINFO,"read_buffer_level","V1720 PLL loss lock Board:%d (vmeAcq=0x%x)"
             , itv1720->GetModuleID(), vmeAcq);
      itv1720->ReadReg(V1720_VME_STATUS, &vmeStat);
    }
  }
  
  // Set ODB flag if PLL lock lost
  if (PLLLockLossID > -1){
    char Path[255];
    sprintf(Path,"/DEAP Alarm/PLL Loss FE0%d",get_frontend_index());
    db_set_value(hDB, 0, Path, &(PLLLockLossID), sizeof(INT), 1, TID_INT);
    // PLL loss lock reset by the VME_STATUS read!
  }
  printf(" | ");  
  return bk_size(pevent);
}
/* emacs
 * Local Variables:
 * mode:C
 * mode:font-lock
 * tab-width: 2
 * c-basic-offset: 2
 * End:
 */

