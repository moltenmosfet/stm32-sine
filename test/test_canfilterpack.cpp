/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2026 Molten MOSFET
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

/* T9 (F12): Stm32Can::PackFilters (libopeninv/src/canfilterpack.cpp) had
 * three bugs:
 *   1. the tail flush passed extIdIndex to SetFilterBankMask instead of
 *      idMaskIndex, so when both a masked-ID remainder and an extended-ID
 *      remainder existed, the extended-ID bank was silently never
 *      programmed;
 *   2. the in-loop mask-bank flush triggered after one id/mask pair
 *      (EXT_IDS_PER_BANK == 2) instead of two pairs (IDS_PER_BANK == 4),
 *      wasting half of every mask bank;
 *   3. a half-filled bank left its unused slot(s) as zero, which is a live
 *      filter entry -- id 0 for list banks, (id 0, mask 0x7FF) for mask
 *      banks -- opening reception for CAN ID 0x000.
 *
 * These tests drive the real PackFilters directly (it is compiled from
 * canfilterpack.cpp, split out of stm32_can.cpp because it needs no
 * hardware register access) and inspect what it told the stubbed
 * can_filter_*_init functions to program. */

#include "stm32_can.h"
#include "stub_libopencm3.h"
#include "test.h"
#include <cstdint>
#include <vector>
#include <algorithm>

class CanFilterPackTest: public UnitTest
{
   public:
      CanFilterPackTest(const std::list<VoidFunction>* cases): UnitTest(cases) {}
      void TestCaseSetup() override { stub_reset_filters(); }
};

/* Decode helpers: the stub records exactly what was passed to the
 * libopencm3 init call, i.e. already encoded the way ConfigureFilters
 * encodes it (left-aligned for standard IDs, shifted+flagged for
 * extended). */
static uint32_t DecodeStd(uint32_t encoded) { return encoded >> 5; }
static uint32_t DecodeExt(uint32_t encoded) { return encoded >> 3; }

static bool Contains(const std::vector<uint32_t>& v, uint32_t id)
{
   return std::find(v.begin(), v.end(), id) != v.end();
}

static void TestOneMasked()
{
   uint32_t ids[]   = { 0x100 };
   uint32_t masks[] = { 0x7F0 };

   Stm32Can::PackFilters(ids, masks, 1, 0);

   ASSERT(stub_filter_call_count == 1);
   ASSERT(stub_filter_calls[0].kind == STUB_FILTER_MASK16);
   ASSERT(stub_filter_calls[0].nr == 0);

   uint32_t id1   = DecodeStd(stub_filter_calls[0].args[0]);
   uint32_t mask1 = DecodeStd(stub_filter_calls[0].args[1]);
   uint32_t id2   = DecodeStd(stub_filter_calls[0].args[2]);
   uint32_t mask2 = DecodeStd(stub_filter_calls[0].args[3]);

   ASSERT(id1 == 0x100 && mask1 == 0x7F0);
   //half-filled bank: pair 2 must be a duplicate of pair 1, not (0, 0x7FF)
   ASSERT(id2 == 0x100 && mask2 == 0x7F0);
   ASSERT(!(id2 == 0 && mask2 == 0x7FF));
}

static void TestOneMaskedOneExtended()
{
   uint32_t ids[]   = { 0x100, 0x1FFFF01 };
   uint32_t masks[] = { 0x7F0, 0 };

   Stm32Can::PackFilters(ids, masks, 2, 0);

   //the wrong-variable tail-flush bug dropped the extended bank whenever a
   //masked remainder was flushed first -- this is the direct regression test
   ASSERT(stub_filter_call_count == 2);

   bool sawMask = false, sawExt = false;
   uint32_t extId = 0;

   for (int i = 0; i < stub_filter_call_count; i++)
   {
      if (stub_filter_calls[i].kind == STUB_FILTER_MASK16)
      {
         sawMask = true;
         ASSERT(DecodeStd(stub_filter_calls[i].args[0]) == 0x100);
      }
      else if (stub_filter_calls[i].kind == STUB_FILTER_LIST32)
      {
         sawExt = true;
         extId = DecodeExt(stub_filter_calls[i].args[0]);
         uint32_t extId2 = DecodeExt(stub_filter_calls[i].args[1]);
         //half-filled 32-bit list bank: duplicate the single real entry
         ASSERT(extId2 == extId);
      }
   }

   ASSERT(sawMask);
   ASSERT(sawExt);
   ASSERT(extId == 0x1FFFF01);
}

static void TestFiveStandard()
{
   uint32_t ids[]   = { 0x10, 0x20, 0x30, 0x40, 0x50 };
   uint32_t masks[] = { 0, 0, 0, 0, 0 };

   Stm32Can::PackFilters(ids, masks, 5, 0);

   ASSERT(stub_filter_call_count == 2);
   ASSERT(stub_filter_calls[0].kind == STUB_FILTER_LIST16);
   ASSERT(stub_filter_calls[0].nr == 0);
   ASSERT(stub_filter_calls[1].kind == STUB_FILTER_LIST16);
   ASSERT(stub_filter_calls[1].nr == 1);

   std::vector<uint32_t> seen;
   for (int c = 0; c < stub_filter_call_count; c++)
   {
      for (int i = 0; i < 4; i++)
      {
         uint32_t id = DecodeStd(stub_filter_calls[c].args[i]);
         ASSERT(id != 0); //no phantom ID-0 slot
         seen.push_back(id);
      }
   }

   for (uint32_t id : ids)
      ASSERT(Contains(seen, id));

   //the second bank is half-filled (one real id, 0x50): the other three
   //slots must repeat it, not fall back to 0
   ASSERT(DecodeStd(stub_filter_calls[1].args[0]) == 0x50);
   ASSERT(DecodeStd(stub_filter_calls[1].args[1]) == 0x50);
   ASSERT(DecodeStd(stub_filter_calls[1].args[2]) == 0x50);
   ASSERT(DecodeStd(stub_filter_calls[1].args[3]) == 0x50);
}

static void TestMixedThreeStdTwoMaskedOneExtended()
{
   uint32_t ids[]   = { 0x11, 0x22, 0x33, 0x200, 0x300, 0x1FFFF55 };
   uint32_t masks[] = { 0,    0,    0,    0x7F0, 0x7F0, 0 };

   Stm32Can::PackFilters(ids, masks, 6, 0);

   ASSERT(stub_filter_call_count == 3);

   bool sawMask = false, sawList = false, sawExt = false;

   for (int c = 0; c < stub_filter_call_count; c++)
   {
      const stub_filter_call& call = stub_filter_calls[c];

      if (call.kind == STUB_FILTER_MASK16)
      {
         sawMask = true;
         //two registered masked IDs exactly fill this bank -- no duplication
         uint32_t id1 = DecodeStd(call.args[0]), mask1 = DecodeStd(call.args[1]);
         uint32_t id2 = DecodeStd(call.args[2]), mask2 = DecodeStd(call.args[3]);
         ASSERT((id1 == 0x200 && id2 == 0x300) || (id1 == 0x300 && id2 == 0x200));
         ASSERT(mask1 == 0x7F0 && mask2 == 0x7F0);
      }
      else if (call.kind == STUB_FILTER_LIST16)
      {
         sawList = true;
         std::vector<uint32_t> bankIds;
         for (int i = 0; i < 4; i++)
         {
            uint32_t id = DecodeStd(call.args[i]);
            ASSERT(id != 0); //no phantom ID-0 slot
            bankIds.push_back(id);
         }
         ASSERT(Contains(bankIds, (uint32_t)0x11));
         ASSERT(Contains(bankIds, (uint32_t)0x22));
         ASSERT(Contains(bankIds, (uint32_t)0x33));
         //half-filled (3 real ids): the 4th slot must repeat the last real id
         ASSERT(bankIds[3] == 0x33);
      }
      else if (call.kind == STUB_FILTER_LIST32)
      {
         sawExt = true;
         uint32_t id1 = DecodeExt(call.args[0]);
         uint32_t id2 = DecodeExt(call.args[1]);
         ASSERT(id1 == 0x1FFFF55 && id2 == 0x1FFFF55);
      }
   }

   ASSERT(sawMask);
   ASSERT(sawList);
   ASSERT(sawExt);
}

REGISTER_TEST(CanFilterPackTest, TestOneMasked, TestOneMaskedOneExtended,
              TestFiveStandard, TestMixedThreeStdTwoMaskedOneExtended);
