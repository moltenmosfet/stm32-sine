/*
 * This file is part of the stm32-... project.
 *
 * Copyright (C) 2021 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "canhardware.h"
#include "my_math.h"
#include "params.h"
#include "errormessage.h"
#include "inc_encoder.h"
#include "digio.h"
#include "hwdefs.h"
#include "pwmgeneration.h"
#include "vehiclecontrol.h"
#include "throttle.h"
#include "stub_canhardware.h"
#include "test.h"

static uint32_t crc32_word(uint32_t Crc, uint32_t Data);

static uint32_t crc = 0;
static uint32_t rtc = 0;
static uint32_t speed = 0;
static ERROR_MESSAGE_NUM errorMessage;

class VCUTest: public UnitTest
{
   public:
      VCUTest(const std::list<VoidFunction>* cases): UnitTest(cases) {}
      virtual void TestCaseSetup();
};

static void FillInCanData(uint32_t* data, uint32_t pot, uint32_t pot2, uint32_t canio, uint32_t cruisespeed, uint32_t regenPreset, uint32_t seq)
{
   uint32_t crc = 0xFFFFFFFF;
   data[0] = pot| (pot2 << 12) | (canio << 24) | (seq << 30);
   data[1] = cruisespeed | (seq << 14) | (regenPreset << 16);
   crc = crc32_word(crc, data[0]);
   crc = crc32_word(crc, data[1]);
   data[1] |= crc << 24;
}

static void CanTest1()
{
   ASSERT(vcuCan != 0);
   ASSERT(vcuCanId == Param::GetInt(Param::controlid));
}

static void CanTest2()
{
   uint32_t data[2];

   FillInCanData(data, 100, 200, CAN_IO_FWD, 1000, 50, 1);

   vcuCan->HandleRx(vcuCanId, data, 8);
   VehicleControl::GetDigInputs();
   VehicleControl::ProcessThrottle();

   ASSERT(Param::GetBool(Param::din_forward));
   ASSERT(!Param::GetBool(Param::din_reverse));
   ASSERT(Param::GetInt(Param::pot) == 0); //pot=0 because throtmode is not CAN
   ASSERT(Param::GetInt(Param::pot2) == 0); //pot2=0 because throtmode is not CAN
   ASSERT(Param::GetInt(Param::regenpreset) == 100); //Overwritten by "ADC" value
}

static void CanTest3()
{
   uint32_t data[2];

   FillInCanData(data, 100, 200, CAN_IO_START, 1000, 50, 1);
   Param::SetInt(Param::potmode, POTMODE_DUALCHANNEL | POTMODE_CAN);

   vcuCan->HandleRx(vcuCanId, data, 8);
   VehicleControl::GetDigInputs();
   VehicleControl::ProcessThrottle();

   ASSERT(Param::GetBool(Param::din_start));
   ASSERT(!Param::GetBool(Param::din_forward));
   ASSERT(Param::GetInt(Param::pot) == 100); //pot=0 because throtmode is not CAN
   ASSERT(Param::GetInt(Param::pot2) == 200); //pot2=0 because throtmode is not CAN
   ASSERT(Param::GetInt(Param::regenpreset) == 50);
}

static void TestCanSeqError1()
{
   uint32_t data[2];

   Param::SetInt(Param::potmode, POTMODE_DUALCHANNEL | POTMODE_CAN);
   FillInCanData(data, 100, 200, CAN_IO_START, 1000, 60, 1);
   vcuCan->HandleRx(vcuCanId, data, 8);
   //call again with same sequence counter -> triggers an error message
   FillInCanData(data, 100, 200, CAN_IO_START, 1000, 60, 1);
   vcuCan->HandleRx(vcuCanId, data, 8);

   ASSERT(errorMessage == ERR_CANCOUNTER);
   ASSERT(Param::GetInt(Param::regenpreset) == 60); //Only after 5 errors will this be reset to 0
}

static void TestCanSeqError2()
{
   uint32_t data[2];

   Param::SetInt(Param::potmode, POTMODE_DUALCHANNEL | POTMODE_CAN);

   for (int i = 0; i < 6; i++)
   {
      //Call 6 times with same sequence counter -> will trigger unrecoverable error
      FillInCanData(data, 100, 200, CAN_IO_START, 1000, 60, 1);
      vcuCan->HandleRx(vcuCanId, data, 8);
   }

   //Now simulate 500ms or 50 rtc ticks passed
   rtc = 51;
   VehicleControl::ProcessThrottle();
   VehicleControl::GetDigInputs();

   ASSERT(errorMessage == ERR_CANTIMEOUT);
   ASSERT(Param::GetInt(Param::regenpreset) == 0);
   ASSERT(!Param::GetBool(Param::din_start));
   ASSERT(Param::GetInt(Param::potnom) == 0);
}

static void TestCanBrakeLightHysteresis()
{
   uint32_t data[2];

   Throttle::potmin[0] = 0;
   Throttle::potmax[0] = 3500;
   Throttle::potmin[1] = 0;
   Throttle::potmax[1] = 4095;
   Throttle::brkmax = -50;
   Throttle::brknompedal = -50;
   Throttle::linearity = 1;
   Throttle::regenRamp = 100;
   Throttle::throttleRamp = 100;
   Throttle::throtmax = 100;
   Throttle::throtmin = -100;
   Throttle::maxregentravelhz = 0;
   Throttle::udcmax = 1000;
   Throttle::idcmin = -5000;
   Throttle::idckp = 1;

   Param::SetInt(Param::potmode, POTMODE_CAN);
   Param::SetFloat(Param::brklightout, -10);

   FillInCanData(data, 500, 0, CAN_IO_FWD | CAN_IO_BRAKE, 0, 100, 1);
   vcuCan->HandleRx(vcuCanId, data, 8);
   VehicleControl::GetDigInputs();
   VehicleControl::ProcessThrottle();
   ASSERT(Param::GetBool(Param::dout_brake));

   FillInCanData(data, 700, 0, CAN_IO_FWD, 0, 50, 2);
   vcuCan->HandleRx(vcuCanId, data, 8);
   VehicleControl::GetDigInputs();
   VehicleControl::ProcessThrottle();
   // Input chosen to stay between on-threshold (-10) and off-threshold (-8) -> light must stay on
   ASSERT(Param::GetBool(Param::dout_brake));

   FillInCanData(data, 3500, 0, CAN_IO_FWD, 0, 50, 3);
   vcuCan->HandleRx(vcuCanId, data, 8);
   VehicleControl::GetDigInputs();
   VehicleControl::ProcessThrottle();
   ASSERT(!Param::GetBool(Param::dout_brake));
}

// ---------------------------------------------------------------------------
// T24: dual-channel (dual-pot) selection coverage.
//
// GetUserThrottleCommand()'s POTMODE_DUALCHANNEL block (vehiclecontrol.cpp,
// the inRange1/inRange2 four-way select) is the successor to upstream's
// deleted Throttle::CheckDualThrottle cross-channel agreement check, and had
// zero coverage (T23 residual 2). These cases drive all four branches through
// the public ProcessThrottle() path and lock the selection + error-posting
// behaviour, including the asymmetry the audit flagged: a bad channel 1 is
// NOT a silent fallback -- the !inRange1 guard upstream of the select posts
// ERR_THROTTLE1 + err_out unconditionally, while a bad channel 2 posts
// ERR_THROTTLE2 from inside the select. Both are locked here.
//
// Setup makes the whole ProcessThrottle chain a pass-through for a positive
// command: brknom=0 + linearity=1 make CalcThrottle the identity, ramp rate
// 200 spans the full [-100,100] span in one call (so the private, persistent
// throttleRamped state can't perturb the golden), and every derate
// (bms/udc/idc/freq/accel/temp) is configured wide enough not to bite. So
// Param::potnom == the selected channel's DigitsToPercent value.
static void SetupDualChannelPassthrough()
{
   Param::SetInt(Param::potmode, POTMODE_DUALCHANNEL | POTMODE_CAN);
   Param::SetFloat(Param::regentravel, 0); // -> brknom=0 via UpdateDynamicRegenTravel
   Param::SetFloat(Param::udc, 400);
   Param::SetFloat(Param::idc, 0);
   Param::SetFloat(Param::fstat, 0);
   Param::SetFloat(Param::tmphs, 0);
   Param::SetFloat(Param::tmphsmax, 100);
   Param::SetFloat(Param::tmpm, 0);
   Param::SetFloat(Param::tmpmmax, 100);

   Throttle::linearity = 1;       // identity CalcThrottle for potnom > brknom
   Throttle::maxregentravelhz = 0;
   Throttle::brkmax = -50;
   Throttle::brknompedal = -50;
   Throttle::throtmax = 100;
   Throttle::throtmin = -100;
   Throttle::throttleRamp = 200;  // one-call convergence, ramp-state independent
   Throttle::regenRamp = 200;
   Throttle::udcmin = 0;
   Throttle::udcmax = 1000;
   Throttle::idcmin = -5000;
   Throttle::idcmax = 1000;
   Throttle::idckp = 1;
   Throttle::fmax = 100000;

   rtc = 100; // keep (rtc - lastCanRxTime) below CAN_TIMEOUT after HandleRx
}

// Drive one CAN frame through the dual-channel path with opmode=MOD_RUN so
// PostErrorIfRunning actually fires, then run the throttle pipeline.
static void RunDualChannel(uint32_t pot, uint32_t pot2)
{
   uint32_t data[2];
   FillInCanData(data, pot, pot2, CAN_IO_FWD, 0, 0, 1);
   vcuCan->HandleRx(vcuCanId, data, 8);
   VehicleControl::GetDigInputs();
   Param::SetInt(Param::opmode, MOD_RUN);
   errorMessage = (ERROR_MESSAGE_NUM)-1; // sentinel: detect "no error posted"
   VehicleControl::ProcessThrottle();
}

// Both channels in range: the lower reading wins (MIN-select), regardless of
// which physical channel it is. Proves it is a MIN, not "always channel 1".
static void TestDualBothGoodMinSelect()
{
   SetupDualChannelPassthrough();
   Throttle::potmin[0] = 0; Throttle::potmax[0] = 4000;
   Throttle::potmin[1] = 0; Throttle::potmax[1] = 4000;

   // ch1 = 50%, ch2 = 20% -> select 20%
   RunDualChannel(2000, 800);
   ASSERT(ABS(Param::GetFloat(Param::potnom) - 20.0f) < 0.01f);

   // ch1 = 20%, ch2 = 50% -> still select the lower, 20% (now from ch1)
   RunDualChannel(800, 2000);
   ASSERT(ABS(Param::GetFloat(Param::potnom) - 20.0f) < 0.01f);
}

// Channel 1 good, channel 2 out of range: use channel 1 as-is, and the select
// posts ERR_THROTTLE2 (channel 1 is fine, so no ERR_THROTTLE1 upstream).
static void TestDualCh1GoodCh2Bad()
{
   SetupDualChannelPassthrough();
   Throttle::potmin[0] = 0; Throttle::potmax[0] = 4000; // ch1 in-range window
   Throttle::potmin[1] = 0; Throttle::potmax[1] = 1000; // ch2 narrow window

   RunDualChannel(2000, 3000); // ch1=50%, ch2=3000 > 1000+SLACK -> out of range
   ASSERT(ABS(Param::GetFloat(Param::potnom) - 50.0f) < 0.01f);
   ASSERT(errorMessage == ERR_THROTTLE2);
}

// Channel 1 out of range, channel 2 good: fall back to channel 2's value. The
// fallback is NOT silent -- the !inRange1 guard already set err_out and posted
// ERR_THROTTLE1; the select posts nothing further, so ERR_THROTTLE1 stands.
// (Asymmetric vs the ch2-bad case, which raises ERR_THROTTLE2 -- per-channel
// error ids, intentional. Locked here as characterization; behaviour unchanged.)
static void TestDualCh1BadCh2Good()
{
   SetupDualChannelPassthrough();
   Throttle::potmin[0] = 0; Throttle::potmax[0] = 1000; // ch1 narrow window
   Throttle::potmin[1] = 0; Throttle::potmax[1] = 4000; // ch2 in-range window

   RunDualChannel(3000, 2000); // ch1=3000 out of range, ch2=50%
   ASSERT(ABS(Param::GetFloat(Param::potnom) - 50.0f) < 0.01f);
   ASSERT(errorMessage == ERR_THROTTLE1); // NOT ERR_THROTTLE2: the flagged asymmetry
}

// Both channels out of range: inhibit movement (return 0). ERR_THROTTLE1 fires
// from the upstream guard, then the select's else-branch posts ERR_THROTTLE2,
// so the last-posted error is ERR_THROTTLE2.
static void TestDualBothBad()
{
   SetupDualChannelPassthrough();
   Throttle::potmin[0] = 0; Throttle::potmax[0] = 1000;
   Throttle::potmin[1] = 0; Throttle::potmax[1] = 1000;

   RunDualChannel(3000, 3000); // both out of range
   ASSERT(Param::GetFloat(Param::potnom) == 0);
   ASSERT(errorMessage == ERR_THROTTLE2);
}

// ---------------------------------------------------------------------------
// T25 [FORK]: dual-pot cross-channel plausibility fault (ruling 2026-07-23, b).
// When POTMODE_DUALCHANNEL and BOTH channels are individually in range but
// disagree by more than `potdiffmax` percentage points, the command stays the
// MIN-select (unchanged) but a DEDICATED fault ERR_THROTTLEDIFF is raised
// (distinct from the per-channel ERR_THROTTLE1/2 range faults) so a
// stuck-but-in-range pot is never silent. potdiffmax == 0 disables the check
// (shipped default),
// making the default binary behaviourally a no-op. TestCaseSetup's LoadDefaults
// resets potdiffmax to 0 before each case; cases that exercise the check enable
// it explicitly. Both channels use the [0,4000] window so DigitsToPercent gives
// percent == pot/40.
// ---------------------------------------------------------------------------

// Disagreement below tolerance: no fault, MIN-select unchanged.
static void TestDualDisagreeBelowTol()
{
   SetupDualChannelPassthrough();
   Throttle::potmin[0] = 0; Throttle::potmax[0] = 4000;
   Throttle::potmin[1] = 0; Throttle::potmax[1] = 4000;
   Param::SetFloat(Param::potdiffmax, 10);

   RunDualChannel(2000, 1800); // ch1=50%, ch2=45%, diff=5 <= 10 -> plausible
   ASSERT(ABS(Param::GetFloat(Param::potnom) - 45.0f) < 0.01f); // MIN unchanged
   ASSERT(errorMessage == (ERROR_MESSAGE_NUM)-1);               // no fault posted
}

// Disagreement above tolerance: ERR_THROTTLEDIFF raised AND MIN still commanded.
static void TestDualDisagreeAboveTol()
{
   SetupDualChannelPassthrough();
   Throttle::potmin[0] = 0; Throttle::potmax[0] = 4000;
   Throttle::potmin[1] = 0; Throttle::potmax[1] = 4000;
   Param::SetFloat(Param::potdiffmax, 10);

   RunDualChannel(2000, 800); // ch1=50%, ch2=20%, diff=30 > 10 -> implausible
   ASSERT(ABS(Param::GetFloat(Param::potnom) - 20.0f) < 0.01f); // MIN-select KEPT
   ASSERT(errorMessage == ERR_THROTTLEDIFF);                    // dedicated fault id
}

// Boundary: the comparison is strict (> tolerance), mirroring upstream's
// diff > 10. A diff exactly equal to the tolerance is plausible; one count
// tighter and the same readings trip.
static void TestDualPlausBoundary()
{
   SetupDualChannelPassthrough();
   Throttle::potmin[0] = 0; Throttle::potmax[0] = 4000;
   Throttle::potmin[1] = 0; Throttle::potmax[1] = 4000;

   // diff == tolerance -> plausible, no fault, MIN commanded.
   Param::SetFloat(Param::potdiffmax, 10);
   RunDualChannel(2000, 1600); // ch1=50%, ch2=40%, diff=10, tol=10
   ASSERT(ABS(Param::GetFloat(Param::potnom) - 40.0f) < 0.01f);
   ASSERT(errorMessage == (ERROR_MESSAGE_NUM)-1);

   // Same readings, tolerance one count tighter -> diff (10) > tol (9) -> fault.
   Param::SetFloat(Param::potdiffmax, 9);
   RunDualChannel(2000, 1600);
   ASSERT(ABS(Param::GetFloat(Param::potnom) - 40.0f) < 0.01f); // MIN still kept
   ASSERT(errorMessage == ERR_THROTTLEDIFF);
}

// Disabled default (potdiffmax == 0): a large disagreement raises NO
// plausibility fault and the MIN-select is bit-identical to pre-T25 behaviour.
static void TestDualPlausDisabledDefault()
{
   SetupDualChannelPassthrough();
   Throttle::potmin[0] = 0; Throttle::potmax[0] = 4000;
   Throttle::potmin[1] = 0; Throttle::potmax[1] = 4000;
   Param::SetFloat(Param::potdiffmax, 0); // explicit; also the LoadDefaults value

   RunDualChannel(2000, 0); // ch1=50%, ch2=0%, diff=50 but check disabled
   ASSERT(Param::GetFloat(Param::potnom) == 0);       // MIN-select unchanged
   ASSERT(errorMessage == (ERROR_MESSAGE_NUM)-1);     // no plausibility fault
}

// Interaction with the per-channel range faults: the plausibility check lives
// only inside the both-in-range branch, so a channel that is out of range
// short-circuits to the existing range-fault path even with the check enabled.
// The dedicated ERR_THROTTLEDIFF id is the whole point -- a range fault and a
// plausibility fault must be DISTINGUISHABLE. Here ch2 is out of range with the
// check ON: we get ERR_THROTTLE2 (the range fault), NOT ERR_THROTTLEDIFF.
static void TestDualPlausRangeFaultInteraction()
{
   // The two fault classes are distinct error ids (dedicated id, not a reuse).
   ASSERT(ERR_THROTTLEDIFF != ERR_THROTTLE1);
   ASSERT(ERR_THROTTLEDIFF != ERR_THROTTLE2);

   SetupDualChannelPassthrough();
   Throttle::potmin[0] = 0; Throttle::potmax[0] = 4000; // ch1 in-range window
   Throttle::potmin[1] = 0; Throttle::potmax[1] = 1000; // ch2 narrow window
   Param::SetFloat(Param::potdiffmax, 10);              // check ON, must not fire

   RunDualChannel(2000, 3000); // ch1=50%, ch2 out of range
   ASSERT(ABS(Param::GetFloat(Param::potnom) - 50.0f) < 0.01f);
   ASSERT(errorMessage == ERR_THROTTLE2);        // range fault ...
   ASSERT(errorMessage != ERR_THROTTLEDIFF);     // ... distinguishable from plausibility
}

void VCUTest::TestCaseSetup()
{
   VehicleControl::SetCan(new CanStub());
   Param::LoadDefaults();
   //These cases assume uncalibrated pots: DigitsToPercent returns the
   //degenerate 100 when potmax == potmin, which is the "ADC value" the
   //non-CAN potmode assertions expect. Other suites (ThrottleTest) calibrate
   //these statics, and suite order follows link order -- pin the state here
   //instead of relying on running first.
   Throttle::potmin[0] = Throttle::potmax[0] = 0;
   Throttle::potmin[1] = Throttle::potmax[1] = 0;
}

REGISTER_TEST(VCUTest, CanTest1, CanTest2, CanTest3, TestCanSeqError1, TestCanSeqError2, TestCanBrakeLightHysteresis,
              TestDualBothGoodMinSelect, TestDualCh1GoodCh2Bad, TestDualCh1BadCh2Good, TestDualBothBad,
              TestDualDisagreeBelowTol, TestDualDisagreeAboveTol, TestDualPlausBoundary,
              TestDualPlausDisabledDefault, TestDualPlausRangeFaultInteraction);

/* Stub functions */
extern "C" void crc_reset()
{
   crc = 0xFFFFFFFF;
}

extern "C" uint32_t crc_calculate_block(uint32_t* data, uint32_t len)
{
   while (len--)
      crc = crc32_word(crc, *(data++));
   return crc;
}

extern "C" uint32_t rtc_get_counter_val()
{
   return rtc;
}

/* timer_set_oc_value now lives in stub_libopencm3.c (recording stub, T1) */

extern "C" void spi_setup()
{

}

extern "C" uint16_t gpio_get(uint32_t port, uint16_t pin)
{
   return 0;
}

extern "C" void gpio_set(uint32_t port, uint16_t pin)
{
}

extern "C" void gpio_clear(uint32_t port, uint16_t pin)
{
}

extern "C" uint16_t spi_xfer(uint32_t, uint16_t)
{
   return 0;
}

bool PwmGeneration::Tripped()
{
   return true;
}

DigIo DigIo::err_out;
DigIo DigIo::brk_out;
DigIo DigIo::dcsw_out;
DigIo DigIo::prec_out;
DigIo DigIo::vtg_out;
DigIo DigIo::fan_out;
DigIo DigIo::temp0_out;
DigIo DigIo::cruise_in;
DigIo DigIo::start_in;
DigIo DigIo::brake_in;
DigIo DigIo::mprot_in;
DigIo DigIo::fwd_in;
DigIo DigIo::rev_in;
DigIo DigIo::emcystop_in;
DigIo DigIo::bms_in;
DigIo DigIo::ocur_in;
DigIo DigIo::desat_in;
AnaIn AnaIn::uaux(0);
AnaIn AnaIn::udc(1);
AnaIn AnaIn::tmphs(2);
AnaIn AnaIn::tmpm(3);
AnaIn AnaIn::throttle1(4);
AnaIn AnaIn::throttle2(5);

void DigIo::Configure(uint32_t, uint16_t, PinMode::PinMode)
{

}

uint16_t AnaIn::Get()
{
   return 0;
}

void ErrorMessage::Post(ERROR_MESSAGE_NUM e)
{
   errorMessage = e;
}

uint32_t Encoder::GetSpeed()
{
   return speed;
}

u32fp Encoder::GetRotorFrequency()
{
   return speed;
}

void Encoder::ResetDistance()
{

}

int32_t Encoder::GetDistance()
{
   return 0;
}

int Encoder::GetRotorDirection()
{
   return 1;
}

void Param::Change(Param::PARAM_NUM p)
{

}

const char* errorListString = "";
HWREV hwRev = HW_REV3;
uint16_t AnaIn::values[NUM_SAMPLES*ANA_IN_COUNT];

static uint32_t crc32_word(uint32_t Crc, uint32_t Data)
{
  int i;

  Crc = Crc ^ Data;

  for(i=0; i<32; i++)
    if (Crc & 0x80000000)
      Crc = (Crc << 1) ^ 0x04C11DB7; // Polynomial used in STM32
    else
      Crc = (Crc << 1);

  return(Crc);
}
