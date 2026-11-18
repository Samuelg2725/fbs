#include <stdio.h>
#include <stdint.h>
#include <time.h>

#include "srsran/asn1/rrc.h"

static void save_line(const char* line)
{
  char   ts[32];
  time_t now = time(NULL);
  struct tm when = *localtime(&now);
  strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &when);

  FILE* out = fopen("/home/revector/sib1_cell.txt", "a");
  if (out) {
    fprintf(out, "%s  %s\n", ts, line);
    fclose(out);
  }
  fprintf(stderr, "\n%s  %s\n", ts, line);
  fflush(stderr);
}

/* SIB4: physical cell IDs of neighbours on the SAME carrier frequency.
 * These are the co-channel cells the strongest-only cell search misses. */
static void print_sib4(const asn1::rrc::sib_type4_s& sib4)
{
  char line[256];
  int  used = snprintf(line, sizeof(line), "SIB4 intra-freq neighbours");
  bool any  = false;
  if (sib4.intra_freq_neigh_cell_list_present) {
    for (uint32_t i = 0; i < sib4.intra_freq_neigh_cell_list.size() && used > 0 && (uint32_t)used < sizeof(line); i++) {
      int wrote = snprintf(line + used, sizeof(line) - (uint32_t)used, " %u", (unsigned)sib4.intra_freq_neigh_cell_list[i].pci);
      if (wrote < 0) {
        break;
      }
      used += wrote;
      any = true;
    }
  }
  if (any) {
    save_line(line);
  }
}

/* SIB5: the other carrier frequencies (EARFCNs) this cell lists as neighbours,
 * and the neighbour PCIs on each. This is the operator's local channel plan. */
static void print_sib5(const asn1::rrc::sib_type5_s& sib5)
{
  char line[256];
  for (uint32_t c = 0; c < sib5.inter_freq_carrier_freq_list.size(); c++) {
    const asn1::rrc::inter_freq_carrier_freq_info_s& carrier = sib5.inter_freq_carrier_freq_list[c];
    int used = snprintf(line, sizeof(line), "SIB5 inter-freq EARFCN %u", (unsigned)carrier.dl_carrier_freq);
    if (carrier.inter_freq_neigh_cell_list_present) {
      int wrote = snprintf(line + used, sizeof(line) - (uint32_t)used, " PCI");
      if (wrote > 0) {
        used += wrote;
      }
      for (uint32_t i = 0; i < carrier.inter_freq_neigh_cell_list.size() && used > 0 && (uint32_t)used < sizeof(line); i++) {
        wrote = snprintf(line + used, sizeof(line) - (uint32_t)used, " %u", (unsigned)carrier.inter_freq_neigh_cell_list[i].pci);
        if (wrote < 0) {
          break;
        }
        used += wrote;
      }
    }
    save_line(line);
  }
}

/* Walk a SystemInformation message and print any SIB4 / SIB5 it carries.
 * Printed once per pdsch_ue run so the log is not spammed on every SI period. */
static void print_sysinfo(const asn1::rrc::sys_info_s& si)
{
  static int sib4_done = 0;
  static int sib5_done = 0;

  if (si.crit_exts.type() != asn1::rrc::sys_info_s::crit_exts_c_::types::sys_info_r8) {
    return;
  }
  const asn1::rrc::sys_info_r8_ies_s& r8 = si.crit_exts.sys_info_r8();
  for (uint32_t i = 0; i < r8.sib_type_and_info.size(); i++) {
    const asn1::rrc::sys_info_r8_ies_s::sib_type_and_info_item_c_& item = r8.sib_type_and_info[i];
    if (!sib4_done && item.type() == asn1::rrc::sys_info_r8_ies_s::sib_type_and_info_item_c_::types::sib4) {
      print_sib4(item.sib4());
      sib4_done = 1;
    } else if (!sib5_done && item.type() == asn1::rrc::sys_info_r8_ies_s::sib_type_and_info_item_c_::types::sib5) {
      print_sib5(item.sib5());
      sib5_done = 1;
    }
  }
}

/* The PDSCH payload is packed bytes. nbits is the transport-block size.
 * The result is appended to /home/revector/sib1_cell.txt because the pdsch_ue
 * status display moves the cursor and paints over a line printed on screen. */
extern "C" void srsran_sib1_print(const uint8_t* payload, uint32_t nbits)
{
  static int reported_failure = 0;
  char       line[256];

  if (payload == NULL || nbits < 16) {
    return;
  }

  asn1::rrc::bcch_dl_sch_msg_s msg;
  asn1::cbit_ref               bref(payload, (nbits + 7) / 8);
  if (msg.unpack(bref) != asn1::SRSASN_SUCCESS) {
    if (!reported_failure) {
      snprintf(line, sizeof(line), "CRC passed, parse failed, transport block %u bits", nbits);
      save_line(line);
      reported_failure = 1;
    }
    return;
  }
  if (msg.msg.type() != asn1::rrc::bcch_dl_sch_msg_type_c::types::c1) {
    return;
  }
  /* SystemInformation messages (SIB2..SIBn) ride the same SI-RNTI as SIB1. */
  if (msg.msg.c1().type() == asn1::rrc::bcch_dl_sch_msg_type_c::c1_c_::types::sys_info) {
    print_sysinfo(msg.msg.c1().sys_info());
    return;
  }
  if (msg.msg.c1().type() != asn1::rrc::bcch_dl_sch_msg_type_c::c1_c_::types::sib_type1) {
    return;
  }

  const asn1::rrc::sib_type1_s& sib1 = msg.msg.c1().sib_type1();
  const uint32_t                cell = (uint32_t)sib1.cell_access_related_info.cell_id.to_number();
  const uint32_t                tac  = (uint32_t)sib1.cell_access_related_info.tac.to_number();
  int                           used = snprintf(line,
                      sizeof(line),
                      "SIB1 cell identity %u   eNB %u   sector %u   TAC %u",
                      cell,
                      cell >> 8,
                      cell & 0xffu,
                      tac);
  const auto&                   list = sib1.cell_access_related_info.plmn_id_list;
  for (uint32_t i = 0; i < list.size() && used > 0 && (uint32_t)used < sizeof(line); i++) {
    const asn1::rrc::plmn_id_s& plmn = list[i].plmn_id;
    unsigned                    mcc  = plmn.mcc[0] * 100u + plmn.mcc[1] * 10u + plmn.mcc[2];
    unsigned                    mnc  = 0;
    for (uint32_t d = 0; d < plmn.mnc.size(); d++) {
      mnc = mnc * 10u + plmn.mnc[d];
    }
    int wrote;
    if (plmn.mnc.size() == 2) {
      wrote = snprintf(line + used, sizeof(line) - (uint32_t)used, "   PLMN %03u-%02u", mcc, mnc);
    } else {
      wrote = snprintf(line + used, sizeof(line) - (uint32_t)used, "   PLMN %03u-%03u", mcc, mnc);
    }
    if (wrote < 0) {
      break;
    }
    used += wrote;
  }
  save_line(line);

  /* The schedule lists which SIBs this cell actually broadcasts. SIB2 is always
   * in the first SI message; sib_map_info lists SIB3 and up. This tells us
   * whether SIB4/SIB5 are even on air before we wait for them. */
  static int sched_done = 0;
  if (!sched_done) {
    char sched[256];
    int  s_used = snprintf(sched, sizeof(sched), "SIB1 schedules SIB2");
    for (uint32_t si = 0; si < sib1.sched_info_list.size() && s_used > 0 && (uint32_t)s_used < sizeof(sched); si++) {
      const asn1::rrc::sched_info_s& info = sib1.sched_info_list[si];
      for (uint32_t m = 0; m < info.sib_map_info.size() && s_used > 0 && (uint32_t)s_used < sizeof(sched); m++) {
        int wrote2 = snprintf(sched + s_used, sizeof(sched) - (uint32_t)s_used, " SIB%d", (int)info.sib_map_info[m].to_number());
        if (wrote2 < 0) {
          break;
        }
        s_used += wrote2;
      }
    }
    save_line(sched);
    sched_done = 1;
  }
}
