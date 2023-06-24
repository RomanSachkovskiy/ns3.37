/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Marco Miozzo <marco.miozzo@cttc.es>
 * Modification: Dizhi Zhou <dizhi.zhou@gmail.com>    // modify codes related to downlink scheduler
 */

#include <ns3/boolean.h>
#include <ns3/log.h>
#include <ns3/lte-amc.h>
#include <ns3/lte-vendor-specific-parameters.h>
#include <ns3/math.h>
#include <ns3/pointer.h>
#include <ns3/simulator.h>
#include <ns3/tdmt-ff-mac-scheduler.h>

#include <cfloat>
#include <set>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("TdMtFfMacScheduler");

/// TDMT type 0 allocation RBG
static const int TdMtType0AllocationRbg[4] = {
    10, // RGB size 1
    26, // RGB size 2
    63, // RGB size 3
    110 // RGB size 4
};      // see table 7.1.6.1-1 of 36.213

NS_OBJECT_ENSURE_REGISTERED(TdMtFfMacScheduler);

TdMtFfMacScheduler::TdMtFfMacScheduler()
    : m_cschedSapUser(nullptr),
      m_schedSapUser(nullptr),
      m_nextRntiUl(0)
{
    m_amc = CreateObject<LteAmc>();
    m_cschedSapProvider = new MemberCschedSapProvider<TdMtFfMacScheduler>(this);
    m_schedSapProvider = new MemberSchedSapProvider<TdMtFfMacScheduler>(this);
}

TdMtFfMacScheduler::~TdMtFfMacScheduler()
{
    NS_LOG_FUNCTION(this);
}

void
TdMtFfMacScheduler::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_dlHarqProcessesDciBuffer.clear();
    m_dlHarqProcessesTimer.clear();
    m_dlHarqProcessesRlcPduListBuffer.clear();
    m_dlInfoListBuffered.clear();
    m_ulHarqCurrentProcessId.clear();
    m_ulHarqProcessesStatus.clear();
    m_ulHarqProcessesDciBuffer.clear();
    delete m_cschedSapProvider;
    delete m_schedSapProvider;
}

TypeId
TdMtFfMacScheduler::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::TdMtFfMacScheduler")
            .SetParent<FfMacScheduler>()
            .SetGroupName("Lte")
            .AddConstructor<TdMtFfMacScheduler>()
            .AddAttribute("CqiTimerThreshold",
                          "The number of TTIs a CQI is valid (default 1000 - 1 sec.)",
                          UintegerValue(1000),
                          MakeUintegerAccessor(&TdMtFfMacScheduler::m_cqiTimersThreshold),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("HarqEnabled",
                          "Activate/Deactivate the HARQ [by default is active].",
                          BooleanValue(true),
                          MakeBooleanAccessor(&TdMtFfMacScheduler::m_harqOn),
                          MakeBooleanChecker())
            .AddAttribute("UlGrantMcs",
                          "The MCS of the UL grant, must be [0..15] (default 0)",
                          UintegerValue(0),
                          MakeUintegerAccessor(&TdMtFfMacScheduler::m_ulGrantMcs),
                          MakeUintegerChecker<uint8_t>());
    return tid;
}

void
TdMtFfMacScheduler::SetFfMacCschedSapUser(FfMacCschedSapUser* s)
{
    m_cschedSapUser = s;
}

void
TdMtFfMacScheduler::SetFfMacSchedSapUser(FfMacSchedSapUser* s)
{
    m_schedSapUser = s;
}

FfMacCschedSapProvider*
TdMtFfMacScheduler::GetFfMacCschedSapProvider()
{
    return m_cschedSapProvider;
}

FfMacSchedSapProvider*
TdMtFfMacScheduler::GetFfMacSchedSapProvider()
{
    return m_schedSapProvider;
}

void
TdMtFfMacScheduler::SetLteFfrSapProvider(LteFfrSapProvider* s)
{
    m_ffrSapProvider = s;
}

LteFfrSapUser*
TdMtFfMacScheduler::GetLteFfrSapUser()
{
    return m_ffrSapUser;
}

void
TdMtFfMacScheduler::DoCschedCellConfigReq(
    const struct FfMacCschedSapProvider::CschedCellConfigReqParameters& params)
{
    NS_LOG_FUNCTION(this);
    // Read the subset of parameters used
    m_cschedCellConfig = params;
    m_rachAllocationMap.resize(m_cschedCellConfig.m_ulBandwidth, 0);
    FfMacCschedSapUser::CschedUeConfigCnfParameters cnf;
    cnf.m_result = SUCCESS;
    m_cschedSapUser->CschedUeConfigCnf(cnf);
}

void
TdMtFfMacScheduler::DoCschedUeConfigReq(
    const struct FfMacCschedSapProvider::CschedUeConfigReqParameters& params)
{
    NS_LOG_FUNCTION(this << " RNTI " << params.m_rnti << " txMode "
                         << (uint16_t)params.m_transmissionMode);
    std::map<uint16_t, uint8_t>::iterator it = m_uesTxMode.find(params.m_rnti);
    if (it == m_uesTxMode.end())
    {
        m_uesTxMode.insert(std::pair<uint16_t, double>(params.m_rnti, params.m_transmissionMode));
        // generate HARQ buffers
        m_dlHarqCurrentProcessId.insert(std::pair<uint16_t, uint8_t>(params.m_rnti, 0));
        DlHarqProcessesStatus_t dlHarqPrcStatus;
        dlHarqPrcStatus.resize(8, 0);
        m_dlHarqProcessesStatus.insert(
            std::pair<uint16_t, DlHarqProcessesStatus_t>(params.m_rnti, dlHarqPrcStatus));
        DlHarqProcessesTimer_t dlHarqProcessesTimer;
        dlHarqProcessesTimer.resize(8, 0);
        m_dlHarqProcessesTimer.insert(
            std::pair<uint16_t, DlHarqProcessesTimer_t>(params.m_rnti, dlHarqProcessesTimer));
        DlHarqProcessesDciBuffer_t dlHarqdci;
        dlHarqdci.resize(8);
        m_dlHarqProcessesDciBuffer.insert(
            std::pair<uint16_t, DlHarqProcessesDciBuffer_t>(params.m_rnti, dlHarqdci));
        DlHarqRlcPduListBuffer_t dlHarqRlcPdu;
        dlHarqRlcPdu.resize(2);
        dlHarqRlcPdu.at(0).resize(8);
        dlHarqRlcPdu.at(1).resize(8);
        m_dlHarqProcessesRlcPduListBuffer.insert(
            std::pair<uint16_t, DlHarqRlcPduListBuffer_t>(params.m_rnti, dlHarqRlcPdu));
        m_ulHarqCurrentProcessId.insert(std::pair<uint16_t, uint8_t>(params.m_rnti, 0));
        UlHarqProcessesStatus_t ulHarqPrcStatus;
        ulHarqPrcStatus.resize(8, 0);
        m_ulHarqProcessesStatus.insert(
            std::pair<uint16_t, UlHarqProcessesStatus_t>(params.m_rnti, ulHarqPrcStatus));
        UlHarqProcessesDciBuffer_t ulHarqdci;
        ulHarqdci.resize(8);
        m_ulHarqProcessesDciBuffer.insert(
            std::pair<uint16_t, UlHarqProcessesDciBuffer_t>(params.m_rnti, ulHarqdci));
    }
    else
    {
        (*it).second = params.m_transmissionMode;
    }
}

void
TdMtFfMacScheduler::DoCschedLcConfigReq(
    const struct FfMacCschedSapProvider::CschedLcConfigReqParameters& params)
{
    NS_LOG_FUNCTION(this << " New LC, rnti: " << params.m_rnti);

    std::set<uint16_t>::iterator it;
    for (std::size_t i = 0; i < params.m_logicalChannelConfigList.size(); i++)
    {
        it = m_flowStatsDl.find(params.m_rnti);

        if (it == m_flowStatsDl.end())
        {
            m_flowStatsDl.insert(params.m_rnti);
            m_flowStatsUl.insert(params.m_rnti);
        }
    }
}

void
TdMtFfMacScheduler::DoCschedLcReleaseReq(
    const struct FfMacCschedSapProvider::CschedLcReleaseReqParameters& params)
{
    NS_LOG_FUNCTION(this);
    for (std::size_t i = 0; i < params.m_logicalChannelIdentity.size(); i++)
    {
        std::map<LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator it =
            m_rlcBufferReq.begin();
        std::map<LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator temp;
        while (it != m_rlcBufferReq.end())
        {
            if (((*it).first.m_rnti == params.m_rnti) &&
                ((*it).first.m_lcId == params.m_logicalChannelIdentity.at(i)))
            {
                temp = it;
                it++;
                m_rlcBufferReq.erase(temp);
            }
            else
            {
                it++;
            }
        }
    }
}

void
TdMtFfMacScheduler::DoCschedUeReleaseReq(
    const struct FfMacCschedSapProvider::CschedUeReleaseReqParameters& params)
{
    NS_LOG_FUNCTION(this);

    m_uesTxMode.erase(params.m_rnti);
    m_dlHarqCurrentProcessId.erase(params.m_rnti);
    m_dlHarqProcessesStatus.erase(params.m_rnti);
    m_dlHarqProcessesTimer.erase(params.m_rnti);
    m_dlHarqProcessesDciBuffer.erase(params.m_rnti);
    m_dlHarqProcessesRlcPduListBuffer.erase(params.m_rnti);
    m_ulHarqCurrentProcessId.erase(params.m_rnti);
    m_ulHarqProcessesStatus.erase(params.m_rnti);
    m_ulHarqProcessesDciBuffer.erase(params.m_rnti);
    m_flowStatsDl.erase(params.m_rnti);
    m_flowStatsUl.erase(params.m_rnti);
    m_ceBsrRxed.erase(params.m_rnti);
    std::map<LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator it =
        m_rlcBufferReq.begin();
    std::map<LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator temp;
    while (it != m_rlcBufferReq.end())
    {
        if ((*it).first.m_rnti == params.m_rnti)
        {
            temp = it;
            it++;
            m_rlcBufferReq.erase(temp);
        }
        else
        {
            it++;
        }
    }
    if (m_nextRntiUl == params.m_rnti)
    {
        m_nextRntiUl = 0;
    }
}

void
TdMtFfMacScheduler::DoSchedDlRlcBufferReq(
    const struct FfMacSchedSapProvider::SchedDlRlcBufferReqParameters& params)
{
    NS_LOG_FUNCTION(this << params.m_rnti << (uint32_t)params.m_logicalChannelIdentity);
    // API generated by RLC for updating RLC parameters on a LC (tx and retx queues)

    std::map<LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator it;

    LteFlowId_t flow(params.m_rnti, params.m_logicalChannelIdentity);

    it = m_rlcBufferReq.find(flow);

    if (it == m_rlcBufferReq.end())
    {
        m_rlcBufferReq.insert(
            std::pair<LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>(flow,
                                                                                         params));
    }
    else
    {
        (*it).second = params;
    }
}

void
TdMtFfMacScheduler::DoSchedDlPagingBufferReq(
    const struct FfMacSchedSapProvider::SchedDlPagingBufferReqParameters& params)
{
    NS_LOG_FUNCTION(this);
    NS_FATAL_ERROR("method not implemented");
}

void
TdMtFfMacScheduler::DoSchedDlMacBufferReq(
    const struct FfMacSchedSapProvider::SchedDlMacBufferReqParameters& params)
{
    NS_LOG_FUNCTION(this);
    NS_FATAL_ERROR("method not implemented");
}

int
TdMtFfMacScheduler::GetRbgSize(int dlbandwidth)
{
    for (int i = 0; i < 4; i++)
    {
        if (dlbandwidth < TdMtType0AllocationRbg[i])
        {
            return (i + 1);
        }
    }

    return (-1);
}

unsigned int
TdMtFfMacScheduler::LcActivePerFlow(uint16_t rnti)
{
    std::map<LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator it;
    unsigned int lcActive = 0;
    for (it = m_rlcBufferReq.begin(); it != m_rlcBufferReq.end(); it++)
    {
        if (((*it).first.m_rnti == rnti) && (((*it).second.m_rlcTransmissionQueueSize > 0) ||
                                             ((*it).second.m_rlcRetransmissionQueueSize > 0) ||
                                             ((*it).second.m_rlcStatusPduSize > 0)))
        {
            lcActive++;
        }
        if ((*it).first.m_rnti > rnti)
        {
            break;
        }
    }
    return (lcActive);
}

uint8_t
TdMtFfMacScheduler::HarqProcessAvailability(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);

    std::map<uint16_t, uint8_t>::iterator it = m_dlHarqCurrentProcessId.find(rnti);
    if (it == m_dlHarqCurrentProcessId.end())
    {
        NS_FATAL_ERROR("No Process Id found for this RNTI " << rnti);
    }
    std::map<uint16_t, DlHarqProcessesStatus_t>::iterator itStat =
        m_dlHarqProcessesStatus.find(rnti);
    if (itStat == m_dlHarqProcessesStatus.end())
    {
        NS_FATAL_ERROR("No Process Id Statusfound for this RNTI " << rnti);
    }
    uint8_t i = (*it).second;
    do
    {
        i = (i + 1) % HARQ_PROC_NUM;
    } while (((*itStat).second.at(i) != 0) && (i != (*it).second));
    if ((*itStat).second.at(i) == 0)
    {
        return (true);
    }
    else
    {
        return (false); // return a not valid harq proc id
    }
}

uint8_t
TdMtFfMacScheduler::UpdateHarqProcessId(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << rnti);

    if (m_harqOn == false)
    {
        return (0);
    }

    std::map<uint16_t, uint8_t>::iterator it = m_dlHarqCurrentProcessId.find(rnti);
    if (it == m_dlHarqCurrentProcessId.end())
    {
        NS_FATAL_ERROR("No Process Id found for this RNTI " << rnti);
    }
    std::map<uint16_t, DlHarqProcessesStatus_t>::iterator itStat =
        m_dlHarqProcessesStatus.find(rnti);
    if (itStat == m_dlHarqProcessesStatus.end())
    {
        NS_FATAL_ERROR("No Process Id Statusfound for this RNTI " << rnti);
    }
    uint8_t i = (*it).second;
    do
    {
        i = (i + 1) % HARQ_PROC_NUM;
    } while (((*itStat).second.at(i) != 0) && (i != (*it).second));
    if ((*itStat).second.at(i) == 0)
    {
        (*it).second = i;
        (*itStat).second.at(i) = 1;
    }
    else
    {
        NS_FATAL_ERROR("No HARQ process available for RNTI "
                       << rnti << " check before update with HarqProcessAvailability");
    }

    return ((*it).second);
}

void
TdMtFfMacScheduler::RefreshHarqProcesses()
{
    NS_LOG_FUNCTION(this);

    std::map<uint16_t, DlHarqProcessesTimer_t>::iterator itTimers;
    for (itTimers = m_dlHarqProcessesTimer.begin(); itTimers != m_dlHarqProcessesTimer.end();
         itTimers++)
    {
        for (uint16_t i = 0; i < HARQ_PROC_NUM; i++)
        {
            if ((*itTimers).second.at(i) == HARQ_DL_TIMEOUT)
            {
                // reset HARQ process

                NS_LOG_DEBUG(this << " Reset HARQ proc " << i << " for RNTI " << (*itTimers).first);
                std::map<uint16_t, DlHarqProcessesStatus_t>::iterator itStat =
                    m_dlHarqProcessesStatus.find((*itTimers).first);
                if (itStat == m_dlHarqProcessesStatus.end())
                {
                    NS_FATAL_ERROR("No Process Id Status found for this RNTI "
                                   << (*itTimers).first);
                }
                (*itStat).second.at(i) = 0;
                (*itTimers).second.at(i) = 0;
            }
            else
            {
                (*itTimers).second.at(i)++;
            }
        }
    }
}

void
TdMtFfMacScheduler::DoSchedDlTriggerReq(
    const struct FfMacSchedSapProvider::SchedDlTriggerReqParameters& params)
{
    NS_LOG_FUNCTION(this << " Frame no. " << (params.m_sfnSf >> 4) << " subframe no. "
                         << (0xF & params.m_sfnSf));
    // API generated by RLC for triggering the scheduling of a DL subframe

    // evaluate the relative channel quality indicator for each UE per each RBG
    // (since we are using allocation type 0 the small unit of allocation is RBG)
    // Resource allocation type 0 (see sec 7.1.6.1 of 36.213)


    std::cout << "______________________________" << std::endl << std::endl;
    std::cout << "DL" << std::endl;
    std::cout << "Current Frame: " << (params.m_sfnSf >> 4) << std::endl;
    std::cout << "Current Subframe: " << (0xF & params.m_sfnSf) << std::endl;
    std::cout << "Current size: " << params.m_dlInfoList.size() << std::endl;


    RefreshDlCqiMaps();

    int rbgSize = GetRbgSize(m_cschedCellConfig.m_dlBandwidth);
    int rbgNum = m_cschedCellConfig.m_dlBandwidth / rbgSize;
    std::map<uint16_t, std::vector<uint16_t>> allocationMap; // RBs map per RNTI
    std::vector<bool> rbgMap;                                // global RBGs map
    uint16_t rbgAllocatedNum = 0;
    std::set<uint16_t> rntiAllocated;
    rbgMap.resize(m_cschedCellConfig.m_dlBandwidth / rbgSize, false);
    FfMacSchedSapUser::SchedDlConfigIndParameters ret;

    //   update UL HARQ proc id
    std::map<uint16_t, uint8_t>::iterator itProcId;
    for (itProcId = m_ulHarqCurrentProcessId.begin(); itProcId != m_ulHarqCurrentProcessId.end();
         itProcId++)
    {
        (*itProcId).second = ((*itProcId).second + 1) % HARQ_PROC_NUM;
    }

    // RACH Allocation
    m_rachAllocationMap.resize(m_cschedCellConfig.m_ulBandwidth, 0);
    uint16_t rbStart = 0;
    std::vector<struct RachListElement_s>::iterator itRach;
    for (itRach = m_rachList.begin(); itRach != m_rachList.end(); itRach++)
    {
        NS_ASSERT_MSG(m_amc->GetUlTbSizeFromMcs(m_ulGrantMcs, m_cschedCellConfig.m_ulBandwidth) >
                          (*itRach).m_estimatedSize,
                      " Default UL Grant MCS does not allow to send RACH messages");
        BuildRarListElement_s newRar;
        newRar.m_rnti = (*itRach).m_rnti;
        // DL-RACH Allocation
        // Ideal: no needs of configuring m_dci
        // UL-RACH Allocation
        newRar.m_grant.m_rnti = newRar.m_rnti;
        newRar.m_grant.m_mcs = m_ulGrantMcs;
        uint16_t rbLen = 1;
        uint16_t tbSizeBits = 0;
        // find lowest TB size that fits UL grant estimated size
        while ((tbSizeBits < (*itRach).m_estimatedSize) &&
               (rbStart + rbLen < m_cschedCellConfig.m_ulBandwidth))
        {
            rbLen++;
            tbSizeBits = m_amc->GetUlTbSizeFromMcs(m_ulGrantMcs, rbLen);
        }
        if (tbSizeBits < (*itRach).m_estimatedSize)
        {
            // no more allocation space: finish allocation
            break;
        }
        newRar.m_grant.m_rbStart = rbStart;
        newRar.m_grant.m_rbLen = rbLen;
        newRar.m_grant.m_tbSize = tbSizeBits / 8;
        newRar.m_grant.m_hopping = false;
        newRar.m_grant.m_tpc = 0;
        newRar.m_grant.m_cqiRequest = false;
        newRar.m_grant.m_ulDelay = false;
        NS_LOG_INFO(this << " UL grant allocated to RNTI " << (*itRach).m_rnti << " rbStart "
                         << rbStart << " rbLen " << rbLen << " MCS " << m_ulGrantMcs << " tbSize "
                         << newRar.m_grant.m_tbSize);
        for (uint16_t i = rbStart; i < rbStart + rbLen; i++)
        {
            m_rachAllocationMap.at(i) = (*itRach).m_rnti;
        }

        if (m_harqOn == true)
        {
            // generate UL-DCI for HARQ retransmissions
            UlDciListElement_s uldci;
            uldci.m_rnti = newRar.m_rnti;
            uldci.m_rbLen = rbLen;
            uldci.m_rbStart = rbStart;
            uldci.m_mcs = m_ulGrantMcs;
            uldci.m_tbSize = tbSizeBits / 8;
            uldci.m_ndi = 1;
            uldci.m_cceIndex = 0;
            uldci.m_aggrLevel = 1;
            uldci.m_ueTxAntennaSelection = 3; // antenna selection OFF
            uldci.m_hopping = false;
            uldci.m_n2Dmrs = 0;
            uldci.m_tpc = 0;            // no power control
            uldci.m_cqiRequest = false; // only period CQI at this stage
            uldci.m_ulIndex = 0;        // TDD parameter
            uldci.m_dai = 1;            // TDD parameter
            uldci.m_freqHopping = 0;
            uldci.m_pdcchPowerOffset = 0; // not used

            uint8_t harqId = 0;
            std::map<uint16_t, uint8_t>::iterator itProcId;
            itProcId = m_ulHarqCurrentProcessId.find(uldci.m_rnti);
            if (itProcId == m_ulHarqCurrentProcessId.end())
            {
                NS_FATAL_ERROR("No info find in HARQ buffer for UE " << uldci.m_rnti);
            }
            harqId = (*itProcId).second;
            std::map<uint16_t, UlHarqProcessesDciBuffer_t>::iterator itDci =
                m_ulHarqProcessesDciBuffer.find(uldci.m_rnti);
            if (itDci == m_ulHarqProcessesDciBuffer.end())
            {
                NS_FATAL_ERROR("Unable to find RNTI entry in UL DCI HARQ buffer for RNTI "
                               << uldci.m_rnti);
            }
            (*itDci).second.at(harqId) = uldci;
        }

        rbStart = rbStart + rbLen;
        ret.m_buildRarList.push_back(newRar);
    }
    m_rachList.clear();

    // Process DL HARQ feedback
    RefreshHarqProcesses();
    // retrieve past HARQ retx buffered
    if (m_dlInfoListBuffered.size() > 0)
    {
        if (params.m_dlInfoList.size() > 0)
        {
            NS_LOG_INFO(this << " Received DL-HARQ feedback");
            m_dlInfoListBuffered.insert(m_dlInfoListBuffered.end(),
                                        params.m_dlInfoList.begin(),
                                        params.m_dlInfoList.end());
        }
    }
    else
    {
        if (params.m_dlInfoList.size() > 0)
        {
            m_dlInfoListBuffered = params.m_dlInfoList;
        }
    }
    if (m_harqOn == false)
    {
        // Ignore HARQ feedback
        m_dlInfoListBuffered.clear();
    }
    std::vector<struct DlInfoListElement_s> dlInfoListUntxed;
    for (std::size_t i = 0; i < m_dlInfoListBuffered.size(); i++)
    {
        std::set<uint16_t>::iterator itRnti = rntiAllocated.find(m_dlInfoListBuffered.at(i).m_rnti);
        if (itRnti != rntiAllocated.end())
        {
            // RNTI already allocated for retx
            continue;
        }
        auto nLayers = m_dlInfoListBuffered.at(i).m_harqStatus.size();
        std::vector<bool> retx;
        NS_LOG_INFO(this << " Processing DLHARQ feedback");
        if (nLayers == 1)
        {
            retx.push_back(m_dlInfoListBuffered.at(i).m_harqStatus.at(0) ==
                           DlInfoListElement_s::NACK);
            retx.push_back(false);
        }
        else
        {
            retx.push_back(m_dlInfoListBuffered.at(i).m_harqStatus.at(0) ==
                           DlInfoListElement_s::NACK);
            retx.push_back(m_dlInfoListBuffered.at(i).m_harqStatus.at(1) ==
                           DlInfoListElement_s::NACK);
        }
        if (retx.at(0) || retx.at(1))
        {
            // retrieve HARQ process information
            uint16_t rnti = m_dlInfoListBuffered.at(i).m_rnti;
            uint8_t harqId = m_dlInfoListBuffered.at(i).m_harqProcessId;
            NS_LOG_INFO(this << " HARQ retx RNTI " << rnti << " harqId " << (uint16_t)harqId);
            std::map<uint16_t, DlHarqProcessesDciBuffer_t>::iterator itHarq =
                m_dlHarqProcessesDciBuffer.find(rnti);
            if (itHarq == m_dlHarqProcessesDciBuffer.end())
            {
                NS_FATAL_ERROR("No info find in HARQ buffer for UE " << rnti);
            }

            DlDciListElement_s dci = (*itHarq).second.at(harqId);
            int rv = 0;
            if (dci.m_rv.size() == 1)
            {
                rv = dci.m_rv.at(0);
            }
            else
            {
                rv = (dci.m_rv.at(0) > dci.m_rv.at(1) ? dci.m_rv.at(0) : dci.m_rv.at(1));
            }

            if (rv == 3)
            {
                // maximum number of retx reached -> drop process
                NS_LOG_INFO("Maximum number of retransmissions reached -> drop process");
                std::map<uint16_t, DlHarqProcessesStatus_t>::iterator it =
                    m_dlHarqProcessesStatus.find(rnti);
                if (it == m_dlHarqProcessesStatus.end())
                {
                    NS_LOG_ERROR("No info find in HARQ buffer for UE (might change eNB) "
                                 << m_dlInfoListBuffered.at(i).m_rnti);
                }
                (*it).second.at(harqId) = 0;
                std::map<uint16_t, DlHarqRlcPduListBuffer_t>::iterator itRlcPdu =
                    m_dlHarqProcessesRlcPduListBuffer.find(rnti);
                if (itRlcPdu == m_dlHarqProcessesRlcPduListBuffer.end())
                {
                    NS_FATAL_ERROR("Unable to find RlcPdcList in HARQ buffer for RNTI "
                                   << m_dlInfoListBuffered.at(i).m_rnti);
                }
                for (std::size_t k = 0; k < (*itRlcPdu).second.size(); k++)
                {
                    (*itRlcPdu).second.at(k).at(harqId).clear();
                }
                continue;
            }
            // check the feasibility of retransmitting on the same RBGs
            // translate the DCI to Spectrum framework
            std::vector<int> dciRbg;
            uint32_t mask = 0x1;
            NS_LOG_INFO("Original RBGs " << dci.m_rbBitmap << " rnti " << dci.m_rnti);
            for (int j = 0; j < 32; j++)
            {
                if (((dci.m_rbBitmap & mask) >> j) == 1)
                {
                    dciRbg.push_back(j);
                    NS_LOG_INFO("\t" << j);
                }
                mask = (mask << 1);
            }
            bool free = true;
            for (std::size_t j = 0; j < dciRbg.size(); j++)
            {
                if (rbgMap.at(dciRbg.at(j)) == true)
                {
                    free = false;
                    break;
                }
            }
            if (free)
            {
                // use the same RBGs for the retx
                // reserve RBGs
                for (std::size_t j = 0; j < dciRbg.size(); j++)
                {
                    rbgMap.at(dciRbg.at(j)) = true;
                    NS_LOG_INFO("RBG " << dciRbg.at(j) << " assigned");
                    rbgAllocatedNum++;
                }

                NS_LOG_INFO(this << " Send retx in the same RBGs");
            }
            else
            {
                // find RBGs for sending HARQ retx
                uint8_t j = 0;
                uint8_t rbgId = (dciRbg.at(dciRbg.size() - 1) + 1) % rbgNum;
                uint8_t startRbg = dciRbg.at(dciRbg.size() - 1);
                std::vector<bool> rbgMapCopy = rbgMap;
                while ((j < dciRbg.size()) && (startRbg != rbgId))
                {
                    if (rbgMapCopy.at(rbgId) == false)
                    {
                        rbgMapCopy.at(rbgId) = true;
                        dciRbg.at(j) = rbgId;
                        j++;
                    }
                    rbgId = (rbgId + 1) % rbgNum;
                }
                if (j == dciRbg.size())
                {
                    // find new RBGs -> update DCI map
                    uint32_t rbgMask = 0;
                    for (std::size_t k = 0; k < dciRbg.size(); k++)
                    {
                        rbgMask = rbgMask + (0x1 << dciRbg.at(k));
                        rbgAllocatedNum++;
                    }
                    dci.m_rbBitmap = rbgMask;
                    rbgMap = rbgMapCopy;
                    NS_LOG_INFO(this << " Move retx in RBGs " << dciRbg.size());
                }
                else
                {
                    // HARQ retx cannot be performed on this TTI -> store it
                    dlInfoListUntxed.push_back(m_dlInfoListBuffered.at(i));
                    NS_LOG_INFO(this << " No resource for this retx -> buffer it");
                }
            }
            // retrieve RLC PDU list for retx TBsize and update DCI
            BuildDataListElement_s newEl;
            std::map<uint16_t, DlHarqRlcPduListBuffer_t>::iterator itRlcPdu =
                m_dlHarqProcessesRlcPduListBuffer.find(rnti);
            if (itRlcPdu == m_dlHarqProcessesRlcPduListBuffer.end())
            {
                NS_FATAL_ERROR("Unable to find RlcPdcList in HARQ buffer for RNTI " << rnti);
            }
            for (std::size_t j = 0; j < nLayers; j++)
            {
                if (retx.at(j))
                {
                    if (j >= dci.m_ndi.size())
                    {
                        // for avoiding errors in MIMO transient phases
                        dci.m_ndi.push_back(0);
                        dci.m_rv.push_back(0);
                        dci.m_mcs.push_back(0);
                        dci.m_tbsSize.push_back(0);
                        NS_LOG_INFO(this << " layer " << (uint16_t)j
                                         << " no txed (MIMO transition)");
                    }
                    else
                    {
                        dci.m_ndi.at(j) = 0;
                        dci.m_rv.at(j)++;
                        (*itHarq).second.at(harqId).m_rv.at(j)++;
                        NS_LOG_INFO(this << " layer " << (uint16_t)j << " RV "
                                         << (uint16_t)dci.m_rv.at(j));
                    }
                }
                else
                {
                    // empty TB of layer j
                    dci.m_ndi.at(j) = 0;
                    dci.m_rv.at(j) = 0;
                    dci.m_mcs.at(j) = 0;
                    dci.m_tbsSize.at(j) = 0;
                    NS_LOG_INFO(this << " layer " << (uint16_t)j << " no retx");
                }
            }
            for (std::size_t k = 0; k < (*itRlcPdu).second.at(0).at(dci.m_harqProcess).size(); k++)
            {
                std::vector<struct RlcPduListElement_s> rlcPduListPerLc;
                for (std::size_t j = 0; j < nLayers; j++)
                {
                    if (retx.at(j))
                    {
                        if (j < dci.m_ndi.size())
                        {
                            NS_LOG_INFO(" layer " << (uint16_t)j << " tb size "
                                                  << dci.m_tbsSize.at(j));
                            rlcPduListPerLc.push_back(
                                (*itRlcPdu).second.at(j).at(dci.m_harqProcess).at(k));
                        }
                    }
                    else
                    { // if no retx needed on layer j, push an RlcPduListElement_s object with
                      // m_size=0 to keep the size of rlcPduListPerLc vector = 2 in case of MIMO
                        NS_LOG_INFO(" layer " << (uint16_t)j << " tb size " << dci.m_tbsSize.at(j));
                        RlcPduListElement_s emptyElement;
                        emptyElement.m_logicalChannelIdentity = (*itRlcPdu)
                                                                    .second.at(j)
                                                                    .at(dci.m_harqProcess)
                                                                    .at(k)
                                                                    .m_logicalChannelIdentity;
                        emptyElement.m_size = 0;
                        rlcPduListPerLc.push_back(emptyElement);
                    }
                }

                if (rlcPduListPerLc.size() > 0)
                {
                    newEl.m_rlcPduList.push_back(rlcPduListPerLc);
                }
            }
            newEl.m_rnti = rnti;
            newEl.m_dci = dci;
            (*itHarq).second.at(harqId).m_rv = dci.m_rv;
            // refresh timer
            std::map<uint16_t, DlHarqProcessesTimer_t>::iterator itHarqTimer =
                m_dlHarqProcessesTimer.find(rnti);
            if (itHarqTimer == m_dlHarqProcessesTimer.end())
            {
                NS_FATAL_ERROR("Unable to find HARQ timer for RNTI " << (uint16_t)rnti);
            }
            (*itHarqTimer).second.at(harqId) = 0;
            ret.m_buildDataList.push_back(newEl);
            rntiAllocated.insert(rnti);
        }
        else
        {
            // update HARQ process status
            NS_LOG_INFO(this << " HARQ received ACK for UE " << m_dlInfoListBuffered.at(i).m_rnti);
            std::map<uint16_t, DlHarqProcessesStatus_t>::iterator it =
                m_dlHarqProcessesStatus.find(m_dlInfoListBuffered.at(i).m_rnti);
            if (it == m_dlHarqProcessesStatus.end())
            {
                NS_FATAL_ERROR("No info find in HARQ buffer for UE "
                               << m_dlInfoListBuffered.at(i).m_rnti);
            }
            (*it).second.at(m_dlInfoListBuffered.at(i).m_harqProcessId) = 0;
            std::map<uint16_t, DlHarqRlcPduListBuffer_t>::iterator itRlcPdu =
                m_dlHarqProcessesRlcPduListBuffer.find(m_dlInfoListBuffered.at(i).m_rnti);
            if (itRlcPdu == m_dlHarqProcessesRlcPduListBuffer.end())
            {
                NS_FATAL_ERROR("Unable to find RlcPdcList in HARQ buffer for RNTI "
                               << m_dlInfoListBuffered.at(i).m_rnti);
            }
            for (std::size_t k = 0; k < (*itRlcPdu).second.size(); k++)
            {
                (*itRlcPdu).second.at(k).at(m_dlInfoListBuffered.at(i).m_harqProcessId).clear();
            }
        }
    }
    m_dlInfoListBuffered.clear();
    m_dlInfoListBuffered = dlInfoListUntxed;

    if (rbgAllocatedNum == rbgNum)
    {
        // all the RBGs are already allocated -> exit
        if ((ret.m_buildDataList.size() > 0) || (ret.m_buildRarList.size() > 0))
        {
            m_schedSapUser->SchedDlConfigInd(ret);
        }
        return;
    }

    std::set<uint16_t>::iterator it;
    std::set<uint16_t>::iterator itMax = m_flowStatsDl.end();
    double metricMax = 0.0;
    for (it = m_flowStatsDl.begin(); it != m_flowStatsDl.end(); it++)
    {
        std::set<uint16_t>::iterator itRnti = rntiAllocated.find((*it));
        if ((itRnti != rntiAllocated.end()) || (!HarqProcessAvailability((*it))))
        {
            // UE already allocated for HARQ or without HARQ process available -> drop it
            if (itRnti != rntiAllocated.end())
            {
                NS_LOG_DEBUG(this << " RNTI discared for HARQ tx" << (uint16_t)(*it));
            }
            if (!HarqProcessAvailability((*it)))
            {
                NS_LOG_DEBUG(this << " RNTI discared for HARQ id" << (uint16_t)(*it));
            }

            continue;
        }

        std::map<uint16_t, uint8_t>::iterator itTxMode;
        itTxMode = m_uesTxMode.find((*it));
        if (itTxMode == m_uesTxMode.end())
        {
            NS_FATAL_ERROR("No Transmission Mode info on user " << (*it));
        }
        auto nLayer = TransmissionModesLayers::TxMode2LayerNum((*itTxMode).second);
        std::map<uint16_t, uint8_t>::iterator itCqi = m_p10CqiRxed.find((*it));
        uint8_t wbCqi = 0;
        if (itCqi != m_p10CqiRxed.end())
        {
            wbCqi = (*itCqi).second;
        }
        else
        {
            wbCqi = 1; // lowest value for trying a transmission
        }

        if (wbCqi != 0)
        {
            // CQI == 0 means "out of range" (see table 7.2.3-1 of 36.213)
            if (LcActivePerFlow(*it) > 0)
            {
                // this UE has data to transmit
                double achievableRate = 0.0;
                for (uint8_t k = 0; k < nLayer; k++)
                {
                    uint8_t mcs = 0;
                    mcs = m_amc->GetMcsFromCqi(wbCqi);
                    achievableRate +=
                        ((m_amc->GetDlTbSizeFromMcs(mcs, rbgSize) / 8) / 0.001); // = TB size / TTI

                    NS_LOG_DEBUG(this << " RNTI " << (*it) << " MCS " << (uint32_t)mcs
                                      << " achievableRate " << achievableRate);
                }

                double metric = achievableRate;

                if (metric > metricMax)
                {
                    metricMax = metric;
                    itMax = it;
                }
            } // LcActivePerFlow

        } // cqi

    } // end for m_flowStatsDl

    if (itMax == m_flowStatsDl.end())
    {
        // no UE available for downlink
        NS_LOG_INFO(this << " any UE found");
    }
    else
    {
        // assign all free RBGs to this UE
        std::vector<uint16_t> tempMap;
        for (int i = 0; i < rbgNum; i++)
        {
            NS_LOG_INFO(this << " ALLOCATION for RBG " << i << " of " << rbgNum);
            NS_LOG_DEBUG(this << " ALLOCATION for RBG " << i << " of " << rbgNum);
            if (rbgMap.at(i) == false)
            {
                rbgMap.at(i) = true;
                tempMap.push_back(i);
            } // end for RBG free

        } // end for RBGs
        if (tempMap.size() > 0)
        {
            allocationMap.insert(std::pair<uint16_t, std::vector<uint16_t>>((*itMax), tempMap));
        }
    }

    // generate the transmission opportunities by grouping the RBGs of the same RNTI and
    // creating the correspondent DCIs
    std::map<uint16_t, std::vector<uint16_t>>::iterator itMap = allocationMap.begin();
    int rbAllocated = 0;
    while (itMap != allocationMap.end())
    {
        // create new BuildDataListElement_s for this LC
        BuildDataListElement_s newEl;
        newEl.m_rnti = (*itMap).first;
        // create the DlDciListElement_s
        DlDciListElement_s newDci;
        newDci.m_rnti = (*itMap).first;
        newDci.m_harqProcess = UpdateHarqProcessId((*itMap).first);

        uint16_t lcActives = LcActivePerFlow((*itMap).first);
        NS_LOG_INFO(this << "Allocate user " << newEl.m_rnti << " rbg " << lcActives);
        if (lcActives == 0)
        {
            // Set to max value, to avoid divide by 0 below
            lcActives = (uint16_t)65535; // UINT16_MAX;
        }
        uint16_t RgbPerRnti = (*itMap).second.size();
        std::map<uint16_t, uint8_t>::iterator itCqi;
        itCqi = m_p10CqiRxed.find((*itMap).first);
        std::map<uint16_t, uint8_t>::iterator itTxMode;
        itTxMode = m_uesTxMode.find((*itMap).first);
        if (itTxMode == m_uesTxMode.end())
        {
            NS_FATAL_ERROR("No Transmission Mode info on user " << (*itMap).first);
        }
        auto nLayer = TransmissionModesLayers::TxMode2LayerNum((*itTxMode).second);
        for (uint8_t j = 0; j < nLayer; j++)
        {
            if (itCqi == m_p10CqiRxed.end())
            {
                newDci.m_mcs.push_back(0); // no info on this user -> lowest MCS
            }
            else
            {
                newDci.m_mcs.push_back(m_amc->GetMcsFromCqi((*itCqi).second));
            }

            int tbSize = (m_amc->GetDlTbSizeFromMcs(newDci.m_mcs.at(j), RgbPerRnti * rbgSize) /
                          8); // (size of TB in bytes according to table 7.1.7.2.1-1 of 36.213)


            std::cout << std::endl;
            std::cout << "RNTI: " << (*itMap).first<< std::endl;
            std::cout << "Allocated RB: " << (rbAllocated * rbgSize) + 1 << std::endl;
            std::cout << "BLOCKS: " << RgbPerRnti * rbgSize << std::endl;
            std::cout << "CMS: " << (int) newDci.m_mcs.at(0) << std::endl;
            std::cout << "Layer: " << (int) nLayer << std::endl;
            std::cout << std::endl;


            newDci.m_tbsSize.push_back(tbSize);
        }

        newDci.m_resAlloc = 0; // only allocation type 0 at this stage
        newDci.m_rbBitmap = 0; // TBD (32 bit bitmap see 7.1.6 of 36.213)
        uint32_t rbgMask = 0;
        for (std::size_t k = 0; k < (*itMap).second.size(); k++)
        {
            rbgMask = rbgMask + (0x1 << (*itMap).second.at(k));
            NS_LOG_INFO(this << " Allocated RBG " << (*itMap).second.at(k));
        }
        newDci.m_rbBitmap = rbgMask; // (32 bit bitmap see 7.1.6 of 36.213)

        // create the rlc PDUs -> equally divide resources among actives LCs
        std::map<LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator
            itBufReq;
        for (itBufReq = m_rlcBufferReq.begin(); itBufReq != m_rlcBufferReq.end(); itBufReq++)
        {
            if (((*itBufReq).first.m_rnti == (*itMap).first) &&
                (((*itBufReq).second.m_rlcTransmissionQueueSize > 0) ||
                 ((*itBufReq).second.m_rlcRetransmissionQueueSize > 0) ||
                 ((*itBufReq).second.m_rlcStatusPduSize > 0)))
            {
                std::vector<struct RlcPduListElement_s> newRlcPduLe;
                for (uint8_t j = 0; j < nLayer; j++)
                {
                    RlcPduListElement_s newRlcEl;
                    newRlcEl.m_logicalChannelIdentity = (*itBufReq).first.m_lcId;
                    newRlcEl.m_size = newDci.m_tbsSize.at(j) / lcActives;
                    NS_LOG_INFO(this << " LCID " << (uint32_t)newRlcEl.m_logicalChannelIdentity
                                     << " size " << newRlcEl.m_size << " layer " << (uint16_t)j);
                    newRlcPduLe.push_back(newRlcEl);
                    UpdateDlRlcBufferInfo(newDci.m_rnti,
                                          newRlcEl.m_logicalChannelIdentity,
                                          newRlcEl.m_size);
                    if (m_harqOn == true)
                    {
                        // store RLC PDU list for HARQ
                        std::map<uint16_t, DlHarqRlcPduListBuffer_t>::iterator itRlcPdu =
                            m_dlHarqProcessesRlcPduListBuffer.find((*itMap).first);
                        if (itRlcPdu == m_dlHarqProcessesRlcPduListBuffer.end())
                        {
                            NS_FATAL_ERROR("Unable to find RlcPdcList in HARQ buffer for RNTI "
                                           << (*itMap).first);
                        }
                        (*itRlcPdu).second.at(j).at(newDci.m_harqProcess).push_back(newRlcEl);
                    }
                }
                newEl.m_rlcPduList.push_back(newRlcPduLe);
            }
            if ((*itBufReq).first.m_rnti > (*itMap).first)
            {
                break;
            }
        }
        for (uint8_t j = 0; j < nLayer; j++)
        {
            newDci.m_ndi.push_back(1);
            newDci.m_rv.push_back(0);
        }

        newDci.m_tpc = 1; // 1 is mapped to 0 in Accumulated Mode and to -1 in Absolute Mode

        newEl.m_dci = newDci;

        if (m_harqOn == true)
        {
            // store DCI for HARQ
            std::map<uint16_t, DlHarqProcessesDciBuffer_t>::iterator itDci =
                m_dlHarqProcessesDciBuffer.find(newEl.m_rnti);
            if (itDci == m_dlHarqProcessesDciBuffer.end())
            {
                NS_FATAL_ERROR("Unable to find RNTI entry in DCI HARQ buffer for RNTI "
                               << newEl.m_rnti);
            }
            (*itDci).second.at(newDci.m_harqProcess) = newDci;
            // refresh timer
            std::map<uint16_t, DlHarqProcessesTimer_t>::iterator itHarqTimer =
                m_dlHarqProcessesTimer.find(newEl.m_rnti);
            if (itHarqTimer == m_dlHarqProcessesTimer.end())
            {
                NS_FATAL_ERROR("Unable to find HARQ timer for RNTI " << (uint16_t)newEl.m_rnti);
            }
            (*itHarqTimer).second.at(newDci.m_harqProcess) = 0;
        }

        // ...more parameters -> ignored in this version

        ret.m_buildDataList.push_back(newEl);

        rbAllocated += RgbPerRnti;
        itMap++;
    }                               // end while allocation
    ret.m_nrOfPdcchOfdmSymbols = 1; /// \todo check correct value according the DCIs txed

    std::cout << "______________________________" << std::endl << std::endl;

    m_schedSapUser->SchedDlConfigInd(ret);
}

void
TdMtFfMacScheduler::DoSchedDlRachInfoReq(
    const struct FfMacSchedSapProvider::SchedDlRachInfoReqParameters& params)
{
    NS_LOG_FUNCTION(this);

    m_rachList = params.m_rachList;
}

void
TdMtFfMacScheduler::DoSchedDlCqiInfoReq(
    const struct FfMacSchedSapProvider::SchedDlCqiInfoReqParameters& params)
{
    NS_LOG_FUNCTION(this);

    for (unsigned int i = 0; i < params.m_cqiList.size(); i++)
    {
        if (params.m_cqiList.at(i).m_cqiType == CqiListElement_s::P10)
        {
            NS_LOG_LOGIC("wideband CQI " << (uint32_t)params.m_cqiList.at(i).m_wbCqi.at(0)
                                         << " reported");
            std::map<uint16_t, uint8_t>::iterator it;
            uint16_t rnti = params.m_cqiList.at(i).m_rnti;
            it = m_p10CqiRxed.find(rnti);
            if (it == m_p10CqiRxed.end())
            {
                // create the new entry
                m_p10CqiRxed.insert(std::pair<uint16_t, uint8_t>(
                    rnti,
                    params.m_cqiList.at(i).m_wbCqi.at(0))); // only codeword 0 at this stage (SISO)
                // generate correspondent timer
                m_p10CqiTimers.insert(std::pair<uint16_t, uint32_t>(rnti, m_cqiTimersThreshold));
            }
            else
            {
                // update the CQI value and refresh correspondent timer
                (*it).second = params.m_cqiList.at(i).m_wbCqi.at(0);
                // update correspondent timer
                std::map<uint16_t, uint32_t>::iterator itTimers;
                itTimers = m_p10CqiTimers.find(rnti);
                (*itTimers).second = m_cqiTimersThreshold;
            }
        }
        else if (params.m_cqiList.at(i).m_cqiType == CqiListElement_s::A30)
        {
            // subband CQI reporting high layer configured
            std::map<uint16_t, SbMeasResult_s>::iterator it;
            uint16_t rnti = params.m_cqiList.at(i).m_rnti;
            it = m_a30CqiRxed.find(rnti);
            if (it == m_a30CqiRxed.end())
            {
                // create the new entry
                m_a30CqiRxed.insert(
                    std::pair<uint16_t, SbMeasResult_s>(rnti,
                                                        params.m_cqiList.at(i).m_sbMeasResult));
                m_a30CqiTimers.insert(std::pair<uint16_t, uint32_t>(rnti, m_cqiTimersThreshold));
            }
            else
            {
                // update the CQI value and refresh correspondent timer
                (*it).second = params.m_cqiList.at(i).m_sbMeasResult;
                std::map<uint16_t, uint32_t>::iterator itTimers;
                itTimers = m_a30CqiTimers.find(rnti);
                (*itTimers).second = m_cqiTimersThreshold;
            }
        }
        else
        {
            NS_LOG_ERROR(this << " CQI type unknown");
        }
    }
}

double
TdMtFfMacScheduler::EstimateUlSinr(uint16_t rnti, uint16_t rb)
{
    std::map<uint16_t, std::vector<double>>::iterator itCqi = m_ueCqi.find(rnti);
    if (itCqi == m_ueCqi.end())
    {
        // no cqi info about this UE
        return (NO_SINR);
    }
    else
    {
        // take the average SINR value among the available
        double sinrSum = 0;
        unsigned int sinrNum = 0;
        for (uint32_t i = 0; i < m_cschedCellConfig.m_ulBandwidth; i++)
        {
            double sinr = (*itCqi).second.at(i);
            if (sinr != NO_SINR)
            {
                sinrSum += sinr;
                sinrNum++;
            }
        }
        double estimatedSinr = (sinrNum > 0) ? (sinrSum / sinrNum) : DBL_MAX;
        // store the value
        (*itCqi).second.at(rb) = estimatedSinr;
        return (estimatedSinr);
    }
}

void
TdMtFfMacScheduler::DoSchedUlTriggerReq(
    const struct FfMacSchedSapProvider::SchedUlTriggerReqParameters& params)
{
    NS_LOG_FUNCTION(this << " UL - Frame no. " << (params.m_sfnSf >> 4) << " subframe no. "
                         << (0xF & params.m_sfnSf) << " size " << params.m_ulInfoList.size());


    // Вывод текущего кадра и подкадра
    std::cout << "______________________________" << std::endl << std::endl;
    std::cout << "UL" << std::endl;
    std::cout << "Current Frame: " << (params.m_sfnSf >> 4) << std::endl;
    std::cout << "Current Subframe: " << (0xF & params.m_sfnSf) << std::endl;
    std::cout << "Current size: " << params.m_ulInfoList.size() << std::endl;


    RefreshUlCqiMaps();

    // Generate RBs map
    FfMacSchedSapUser::SchedUlConfigIndParameters ret;
    std::vector<bool> rbMap;
    std::set<uint16_t> rntiAllocated;
    std::vector<uint16_t> rbgAllocationMap;
    // int rbgNum = m_cschedCellConfig.m_ulBandwidth;
    // int rbgSize = 1;
    // update with RACH allocation map
    rbgAllocationMap = m_rachAllocationMap;
    // rbgAllocationMap.resize (m_cschedCellConfig.m_ulBandwidth, 0);
    m_rachAllocationMap.clear();
    m_rachAllocationMap.resize(m_cschedCellConfig.m_ulBandwidth, 0);

    rbMap.resize(m_cschedCellConfig.m_ulBandwidth, false);
    // remove RACH allocation
    for (uint16_t i = 0; i < m_cschedCellConfig.m_ulBandwidth; i++)
    {
        if (rbgAllocationMap.at(i) != 0)
        {
            rbMap.at(i) = true;
            NS_LOG_DEBUG(this << " Allocated for RACH " << i);
        }
    }

    if (m_harqOn == true)
    {
        //   Process UL HARQ feedback
        for (std::size_t i = 0; i < params.m_ulInfoList.size(); i++)
        {
            if (params.m_ulInfoList.at(i).m_receptionStatus == UlInfoListElement_s::NotOk)
            {
                // retx correspondent block: retrieve the UL-DCI
                uint16_t rnti = params.m_ulInfoList.at(i).m_rnti;
                std::map<uint16_t, uint8_t>::iterator itProcId =
                    m_ulHarqCurrentProcessId.find(rnti);
                if (itProcId == m_ulHarqCurrentProcessId.end())
                {
                    NS_LOG_ERROR("No info find in HARQ buffer for UE (might change eNB) " << rnti);
                }
                uint8_t harqId = (uint8_t)((*itProcId).second - HARQ_PERIOD) % HARQ_PROC_NUM;
                NS_LOG_INFO(this << " UL-HARQ retx RNTI " << rnti << " harqId " << (uint16_t)harqId
                                 << " i " << i << " size " << params.m_ulInfoList.size());
                std::map<uint16_t, UlHarqProcessesDciBuffer_t>::iterator itHarq =
                    m_ulHarqProcessesDciBuffer.find(rnti);
                if (itHarq == m_ulHarqProcessesDciBuffer.end())
                {
                    NS_LOG_ERROR("No info find in HARQ buffer for UE (might change eNB) " << rnti);
                    continue;
                }
                UlDciListElement_s dci = (*itHarq).second.at(harqId);
                std::map<uint16_t, UlHarqProcessesStatus_t>::iterator itStat =
                    m_ulHarqProcessesStatus.find(rnti);
                if (itStat == m_ulHarqProcessesStatus.end())
                {
                    NS_LOG_ERROR("No info find in HARQ buffer for UE (might change eNB) " << rnti);
                }
                if ((*itStat).second.at(harqId) >= 3)
                {
                    NS_LOG_INFO("Max number of retransmissions reached (UL)-> drop process");
                    continue;
                }
                bool free = true;
                for (int j = dci.m_rbStart; j < dci.m_rbStart + dci.m_rbLen; j++)
                {
                    if (rbMap.at(j) == true)
                    {
                        free = false;
                        NS_LOG_INFO(this << " BUSY " << j);
                    }
                }
                if (free)
                {
                    // retx on the same RBs
                    for (int j = dci.m_rbStart; j < dci.m_rbStart + dci.m_rbLen; j++)
                    {
                        rbMap.at(j) = true;
                        rbgAllocationMap.at(j) = dci.m_rnti;
                        NS_LOG_INFO("\tRB " << j);
                    }
                    NS_LOG_INFO(this << " Send retx in the same RBs " << (uint16_t)dci.m_rbStart
                                     << " to " << dci.m_rbStart + dci.m_rbLen << " RV "
                                     << (*itStat).second.at(harqId) + 1);
                }
                else
                {
                    NS_LOG_INFO("Cannot allocate retx due to RACH allocations for UE " << rnti);
                    continue;
                }
                dci.m_ndi = 0;
                // Update HARQ buffers with new HarqId
                (*itStat).second.at((*itProcId).second) = (*itStat).second.at(harqId) + 1;
                (*itStat).second.at(harqId) = 0;
                (*itHarq).second.at((*itProcId).second) = dci;
                ret.m_dciList.push_back(dci);
                rntiAllocated.insert(dci.m_rnti);
            }
            else
            {
                NS_LOG_INFO(this << " HARQ-ACK feedback from RNTI "
                                 << params.m_ulInfoList.at(i).m_rnti);
            }
        }
    }

    std::map<uint16_t, uint32_t>::iterator it;
    int nflows = 0;

    for (it = m_ceBsrRxed.begin(); it != m_ceBsrRxed.end(); it++)
    {
        std::set<uint16_t>::iterator itRnti = rntiAllocated.find((*it).first);
        // select UEs with queues not empty and not yet allocated for HARQ
        if (((*it).second > 0) && (itRnti == rntiAllocated.end()))
        {
            nflows++;
        }
    }

    // if (nflows == 0)
    // {
    //     if (ret.m_dciList.size() > 0)
    //     {
    //         m_allocationMaps.insert(
    //             std::pair<uint16_t, std::vector<uint16_t>>(params.m_sfnSf, rbgAllocationMap));
    //         m_schedSapUser->SchedUlConfigInd(ret);
    //     }

    //     return; // no flows to be scheduled
    // }

    // // Divide the remaining resources equally among the active users starting from the subsequent
    // // one served last scheduling trigger
    uint16_t rbPerFlow = 0;
    if (rbPerFlow < 3)
    {
        rbPerFlow = 3; // at least 3 rbg per flow (till available resource) to ensure TxOpportunity
                       // >= 7 bytes
    }
    int rbAllocated = 0;

    // if (m_nextRntiUl != 0)
    // {
    //     for (it = m_ceBsrRxed.begin(); it != m_ceBsrRxed.end(); it++)
    //     {
    //         if ((*it).first == m_nextRntiUl)
    //         {
    //             break;
    //         }
    //     }
    //     if (it == m_ceBsrRxed.end())
    //     {
    //         NS_LOG_ERROR(this << " no user found");
    //     }
    // }
    // else
    // {
    //     it = m_ceBsrRxed.begin();
    //     m_nextRntiUl = (*it).first;
    // }

    // Выбор ресурсных блоков, которые будут назначены абонентам (здесь осуществляется планирование радиоресурсов)
    std::set<uint16_t>::iterator itFlow;
    std::set<uint16_t>::iterator itMax = m_flowStatsUl.end();
    uint16_t rbLen = 1;
    double metricMax = 0.0;
    for (itFlow = m_flowStatsUl.begin(); itFlow != m_flowStatsUl.end(); itFlow++)
    {
        std::set<uint16_t>::iterator itRnti = rntiAllocated.find((*itFlow));
        if ((itRnti != rntiAllocated.end()) || (!HarqProcessAvailability((*itFlow))))
        {
            // UE already allocated for HARQ or without HARQ process available -> drop it
            if (itRnti != rntiAllocated.end())
            {
                NS_LOG_DEBUG(this << " RNTI discared for HARQ tx" << (uint16_t)(*itFlow));
            }
            if (!HarqProcessAvailability((*itFlow)))
            {
                NS_LOG_DEBUG(this << " RNTI discared for HARQ id" << (uint16_t)(*itFlow));
            }

            continue;
        }

        // check first what are channel conditions for this UE, if CQI!=0
        std::map<uint16_t, std::vector<double>>::iterator itCqi;
        itCqi = m_ueCqi.find((*itFlow));
        std::map<uint16_t, uint8_t>::iterator itTxMode;
        itTxMode = m_uesTxMode.find((*itFlow));
        if (itTxMode == m_uesTxMode.end())
        {
            NS_FATAL_ERROR("No Transmission Mode info on user " << (*itFlow));
        }
        auto nLayer = TransmissionModesLayers::TxMode2LayerNum((*itTxMode).second);

        double wbCqi = 0;
        if (itCqi != m_ueCqi.end())
        {
            wbCqi = (*itCqi).second.at(0);
        }
        else
        {
            wbCqi = 1; // lowest value for trying a transmission
        }

        if (wbCqi != 0)
        {
            double achievableRate = 0.0;
            int cqi = 0;
            std::uint8_t mcs;
            // Выбор модуляционно-кодовых схем для абонентов
            for (uint8_t j = 0; j < nLayer; j++)
            {
                if (itCqi == m_ueCqi.end())
                {
                    // no cqi info about this UE
                    mcs = 0; // MCS 0 -> UL-AMC TBD
                }
                else
                {
                    // take the lowest CQI value (worst RB)
                    // NS_ABORT_MSG_IF((*itCqi).second.size() == 0,
                    //                 "CQI of RNTI = " << (*it).first << " has expired");
                    NS_ABORT_MSG_IF((*itCqi).second.size() == 0,
                                    "CQI of RNTI = " << (*itMax) << " has expired");
                    double minSinr = (*itCqi).second.at(0);
                    if (minSinr == NO_SINR)
                    {
                        // minSinr = EstimateUlSinr((*it).first, uldci.m_rbStart);
                        minSinr = EstimateUlSinr((*itMax), 0);
                    }
                    // rbLen = rbgPerRntiLog[(*itMax).first];
                    // rbLen = (i + rbgPerRntiLog[(*itMax).first] < m_cschedCellConfig.m_ulBandwidth) ? rbgPerRntiLog[(*itMax).first] : 1;
                    uint16_t size = 0 + rbLen;
                    // uint16_t size = i + rbLen;
                    for (uint16_t j = 0; j < size; j++)
                    {
                        double sinr = (*itCqi).second.at(j);
                        if (sinr == NO_SINR)
                        {
                            // sinr = EstimateUlSinr((*it).first, i);
                            sinr = EstimateUlSinr((*itMax), j);
                        }
                        if (sinr < minSinr)
                        {
                            minSinr = sinr;
                        }
                    }

                    // rbLen = (i + rbgPerRntiLog[(*itMax).first] <= m_cschedCellConfig.m_ulBandwidth) ? rbgPerRntiLog[(*itMax).first] : 1;

                    // translate SINR -> cqi: WILD ACK: same as DL
                    double s = log2(1 + (std::pow(10, minSinr / 10) / ((-std::log(5.0 * 0.00005)) / 1.5)));
                    cqi = m_amc->GetCqiFromSpectralEfficiency(s);
                    if (cqi == 0)
                    {
                        it++;
                        if (it == m_ceBsrRxed.end())
                        {
                            // restart from the first
                            it = m_ceBsrRxed.begin();
                        }
                        NS_LOG_DEBUG(this << " UE discarded for CQI = 0, RNTI " << (*itMax));
                        // remove UE from allocation map
                        // for (uint16_t j = i; i < i + rbLen; j++)
                        // {
                        //     rbgAllocationMap.at(j) = 0;
                        // }
                        continue; // CQI == 0 means "out of range" (see table 7.2.3-1 of 36.213)
                    }
                    mcs = m_amc->GetMcsFromCqi(cqi);
                }
                achievableRate += ((m_amc->GetUlTbSizeFromMcs(mcs, rbLen) / 8)); // = TB size / TTI
        

                double metric = achievableRate;
                // Выбор абонента с наибольшей метрикой
                if (metric > metricMax)
                {
                    metricMax = metric;
                    itMax = itFlow;
                }
            }
        }
    }

    rbgAllocationMap.resize(m_cschedCellConfig.m_ulBandwidth, 0);
    if (itMax == m_flowStatsUl.end())
    {
        // no UE available for uplink
        return;
    }
    else
    {
        // assign all RBGs to this UE
        for (int i = 0; i < m_cschedCellConfig.m_ulBandwidth; i++)
        {
            rbMap.at(i) = true;
            rbgAllocationMap.at(i) = (*itMax);
        }
    }

    std::map<uint16_t, std::vector<uint16_t>> allocationMap;
    // std::map<uint16_t, fdbetsFlowPerf_t>::iterator itStats;
    for (itFlow = m_flowStatsUl.begin(); itFlow != m_flowStatsUl.end(); itFlow++)
    {
        std::vector<uint16_t> tempVectorOfRBs;
        std::copy_if(rbgAllocationMap.begin(), rbgAllocationMap.end(), std::back_inserter(tempVectorOfRBs), [&](uint16_t UE){ return UE == (*itFlow); });
        allocationMap.insert(
            std::pair<uint16_t, std::vector<uint16_t>>((*itFlow), tempVectorOfRBs));
    }

    // Распределение выделенных радиоресурсов по абонентам
    std::map<uint16_t, std::vector<uint16_t>>::iterator itMap = allocationMap.begin();
    do
    {
        // std::set<uint16_t>::iterator itRnti = rntiAllocated.find((*it).first);
        // std::map<uint16_t, std::vector<uint16_t>>::iterator itMap = allocationMap.find((*it).first);
        rbPerFlow = (*itMap).second.size();
        // if (rbPerFlow == 0) return;
        // if ((itRnti != rntiAllocated.end()) || ((*it).second == 0))
        // {
        //     // UE already allocated for UL-HARQ -> skip it
        //     NS_LOG_DEBUG(this << " UE already allocated in HARQ -> discared, RNTI " << (*it).first);
        //     it++;
        //     if (it == m_ceBsrRxed.end())
        //     {
        //         // restart from the first
        //         it = m_ceBsrRxed.begin();
        //     }
        //     continue;
        // }

        // if (rbAllocated + rbPerFlow - 1 > m_cschedCellConfig.m_ulBandwidth)
        // {
        //     // limit to physical resources last resource assignment
        //     rbPerFlow = m_cschedCellConfig.m_ulBandwidth - rbAllocated;
        //     // at least 3 rbg per flow to ensure TxOpportunity >= 7 bytes
        //     if (rbPerFlow < 3)
        //     {
        //         // terminate allocation
        //         rbPerFlow = 0;
        //     }
        // }

        UlDciListElement_s uldci;
        // uldci.m_rnti = (*it).first;
        uldci.m_rnti = (*itMap).first;
        uldci.m_rbLen = rbPerFlow;
        
        if (rbAllocated >= m_cschedCellConfig.m_ulBandwidth) {
            uldci.m_rbStart = m_cschedCellConfig.m_ulBandwidth - 1;
        } else {
            uldci.m_rbStart = rbAllocated;
        }
        // bool allocated = false;
        NS_LOG_INFO(this << " RB Allocated " << rbAllocated << " rbPerFlow " << rbPerFlow
                         << " flows " << nflows);
        // while ((!allocated) && ((rbAllocated + rbPerFlow - m_cschedCellConfig.m_ulBandwidth) < 1) &&
        //        (rbPerFlow != 0))
        // {
        //     // check availability
        //     bool free = true;
        //     for (int j = rbAllocated; j < rbAllocated + rbPerFlow; j++)
        //     {
        //         if (rbMap.at(j) == true)
        //         {
        //             free = false;
        //             break;
        //         }
        //     }
        //     if (free)
        //     {
        //         uldci.m_rbStart = rbAllocated;

        //         for (int j = rbAllocated; j < rbAllocated + rbPerFlow; j++)
        //         {
        //             rbMap.at(j) = true;
        //             // store info on allocation for managing ul-cqi interpretation
        //             // rbgAllocationMap.at(j) = (*it).first;
        //             rbgAllocationMap.at(j) = uldci.m_rnti;
        //         }
        //         rbAllocated += rbPerFlow;
        //         allocated = true;
        //         break;
        //     }
        //     rbAllocated++;
        //     if (rbAllocated + rbPerFlow - 1 > m_cschedCellConfig.m_ulBandwidth)
        //     {
        //         // limit to physical resources last resource assignment
        //         rbPerFlow = m_cschedCellConfig.m_ulBandwidth - rbAllocated;
        //         // at least 3 rbg per flow to ensure TxOpportunity >= 7 bytes
        //         if (rbPerFlow < 3)
        //         {
        //             // terminate allocation
        //             rbPerFlow = 0;
        //         }
        //     }
        // }
        // if (!allocated)
        // {
        //     // unable to allocate new resource: finish scheduling
        //     m_nextRntiUl = (*it).first;
        //     if (ret.m_dciList.size() > 0)
        //     {
        //         m_schedSapUser->SchedUlConfigInd(ret);
        //     }
        //     m_allocationMaps.insert(
        //         std::pair<uint16_t, std::vector<uint16_t>>(params.m_sfnSf, rbgAllocationMap));
        //     return;
        // }

        // std::map<uint16_t, std::vector<double>>::iterator itCqi = m_ueCqi.find((*it).first);
        std::map<uint16_t, std::vector<double>>::iterator itCqi = m_ueCqi.find(uldci.m_rnti);
        std::map<uint16_t, uint8_t>::iterator itTxMode;
        itTxMode = m_uesTxMode.find((*itMap).first);

        if (itTxMode == m_uesTxMode.end())
        {
            NS_FATAL_ERROR("No Transmission Mode info on user " << (*itMap).first);
        }
        auto nLayer = TransmissionModesLayers::TxMode2LayerNum((*itTxMode).second);

        uint32_t bytesTxed = 0;
        int cqi = 0;
        for (uint8_t j = 0; j < nLayer; j++)
        {
            if (itCqi == m_ueCqi.end())
            {
                // no cqi info about this UE
                uldci.m_mcs = 0; // MCS 0 -> UL-AMC TBD
            }
            else
            {
                // take the lowest CQI value (worst RB)
                // NS_ABORT_MSG_IF((*itCqi).second.size() == 0,
                //                 "CQI of RNTI = " << (*it).first << " has expired");
                NS_ABORT_MSG_IF((*itCqi).second.size() == 0,
                                "CQI of RNTI = " << uldci.m_rnti << " has expired");
                double minSinr = (*itCqi).second.at(uldci.m_rbStart);
                if (minSinr == NO_SINR)
                {
                    // minSinr = EstimateUlSinr((*it).first, uldci.m_rbStart);
                    minSinr = EstimateUlSinr(uldci.m_rnti, uldci.m_rbStart);
                }
                for (uint16_t i = uldci.m_rbStart; i < uldci.m_rbStart + uldci.m_rbLen; i++)
                {
                    double sinr = (*itCqi).second.at(i);
                    if (sinr == NO_SINR)
                    {
                        // sinr = EstimateUlSinr((*it).first, i);
                        sinr = EstimateUlSinr(uldci.m_rnti, i);
                    }
                    if (sinr < minSinr)
                    {
                        minSinr = sinr;
                    }
                }

                // translate SINR -> cqi: WILD ACK: same as DL
                double s = log2(1 + (std::pow(10, minSinr / 10) / ((-std::log(5.0 * 0.00005)) / 1.5)));
                cqi = m_amc->GetCqiFromSpectralEfficiency(s);
                if (cqi == 0)
                {
                    it++;
                    itMap++;
                    if (it == m_ceBsrRxed.end())
                    {
                        // restart from the first
                        it = m_ceBsrRxed.begin();
                    }
                    NS_LOG_DEBUG(this << " UE discarded for CQI = 0, RNTI " << uldci.m_rnti);
                    // remove UE from allocation map
                    for (uint16_t i = uldci.m_rbStart; i < uldci.m_rbStart + uldci.m_rbLen; i++)
                    {
                        rbgAllocationMap.at(i) = 0;
                    }
                    continue; // CQI == 0 means "out of range" (see table 7.2.3-1 of 36.213)
                }
                uldci.m_mcs = m_amc->GetMcsFromCqi(cqi);
            }

            // Вывод данных для построения диаграммы радиоресурсов
            std::cout << std::endl;
            std::cout << "RNTI: " << (*itMap).first<< std::endl;
            std::cout << "Allocated RB: " << (int) uldci.m_rbStart + 1 << std::endl;
            std::cout << "BLOCKS: " << rbPerFlow << std::endl;
            std::cout << "CMS: " << (int) uldci.m_mcs << std::endl;
            std::cout << "Layer: " << (int) nLayer << std::endl;
            std::cout << std::endl;
        
            // Вычисление размера данных, которые может передавать абонент
            if (rbPerFlow == 0) {
                uldci.m_tbSize = 0;
            } else {
                uldci.m_tbSize = (m_amc->GetUlTbSizeFromMcs(uldci.m_mcs, rbPerFlow) / 8);
            }
            bytesTxed += uldci.m_tbSize;
        }

        UpdateUlRlcBufferInfo(uldci.m_rnti, uldci.m_tbSize);
        uldci.m_ndi = 1;
        uldci.m_cceIndex = 0;
        uldci.m_aggrLevel = 1;
        uldci.m_ueTxAntennaSelection = 3; // antenna selection OFF
        uldci.m_hopping = false;
        uldci.m_n2Dmrs = 0;
        uldci.m_tpc = 0;            // no power control
        uldci.m_cqiRequest = false; // only period CQI at this stage
        uldci.m_ulIndex = 0;        // TDD parameter
        uldci.m_dai = 1;            // TDD parameter
        uldci.m_freqHopping = 0;
        uldci.m_pdcchPowerOffset = 0; // not used
        ret.m_dciList.push_back(uldci);
        // store DCI for HARQ_PERIOD
        uint8_t harqId = 0;
        if (m_harqOn == true)
        {
            std::map<uint16_t, uint8_t>::iterator itProcId;
            itProcId = m_ulHarqCurrentProcessId.find(uldci.m_rnti);
            if (itProcId == m_ulHarqCurrentProcessId.end())
            {
                NS_FATAL_ERROR("No info find in HARQ buffer for UE " << uldci.m_rnti);
            }
            harqId = (*itProcId).second;
            std::map<uint16_t, UlHarqProcessesDciBuffer_t>::iterator itDci =
                m_ulHarqProcessesDciBuffer.find(uldci.m_rnti);
            if (itDci == m_ulHarqProcessesDciBuffer.end())
            {
                NS_FATAL_ERROR("Unable to find RNTI entry in UL DCI HARQ buffer for RNTI "
                               << uldci.m_rnti);
            }
            (*itDci).second.at(harqId) = uldci;
            // Update HARQ process status (RV 0)
            std::map<uint16_t, UlHarqProcessesStatus_t>::iterator itStat =
                m_ulHarqProcessesStatus.find(uldci.m_rnti);
            if (itStat == m_ulHarqProcessesStatus.end())
            {
                NS_LOG_ERROR("No info find in HARQ buffer for UE (might change eNB) "
                             << uldci.m_rnti);
            }
            (*itStat).second.at(harqId) = 0;
        }

        NS_LOG_INFO(this << " UE Allocation RNTI " << (*it).first << " startPRB "
                         << (uint32_t)uldci.m_rbStart << " nPRB " << (uint32_t)uldci.m_rbLen
                         << " CQI " << cqi << " MCS " << (uint32_t)uldci.m_mcs << " TBsize "
                         << uldci.m_tbSize << " RbAlloc " << rbAllocated << " harqId "
                         << (uint16_t)harqId);


        // it++;
        // if (it == m_ceBsrRxed.end())
        // {
        //     // restart from the first
        //     it = m_ceBsrRxed.begin();
        // }
        // if ((rbAllocated == m_cschedCellConfig.m_ulBandwidth) || (rbPerFlow == 0))
        // {
        //     // Stop allocation: no more PRBs
        //     m_nextRntiUl = (*it).first;
        //     break;
        // }

        rbAllocated += rbPerFlow;
        itMap++;
    } while (itMap != allocationMap.end());


    // std::map<uint16_t, std::vector<uint16_t>> allocationMap; // RBs map per RNTI
    // std::set<uint16_t>::iterator it;
    // std::set<uint16_t>::iterator itMax = m_flowStatsUl.end();
    // double metricMax = 0.0;

    // for (it = m_flowStatsUl.begin(); it != m_flowStatsUl.end(); it++) 
    // {
    //     std::set<uint16_t>::iterator itRntieeeee = rntiAllocated.find((*it));
    //     if ((itRntieeeee != rntiAllocated.end()) || (!HarqProcessAvailability((*it))))
    //     {
    //         // UE already allocated for HARQ or without HARQ process available -> drop it
    //         if (itRntieeeee != rntiAllocated.end())
    //         {
    //             NS_LOG_DEBUG(this << " RNTI discared for HARQ tx" << (uint16_t)(*it));
    //         }
    //         if (!HarqProcessAvailability((*it)))
    //         {
    //             NS_LOG_DEBUG(this << " RNTI discared for HARQ id" << (uint16_t)(*it));
    //         }

    //         continue;
    //     }

    //     std::map<uint16_t, uint8_t>::iterator itTxMode;
    //     itTxMode = m_uesTxMode.find((*it));
    //     if (itTxMode == m_uesTxMode.end())
    //     {
    //         NS_FATAL_ERROR("No Transmission Mode info on user " << (*it));
    //     }
    //     auto nLayer = TransmissionModesLayers::TxMode2LayerNum((*itTxMode).second);
    //     std::map<uint16_t, uint8_t>::iterator itCqi = m_p10CqiRxed.find((*it));
    //     uint8_t wbCqi = 0;
    //     if (itCqi != m_p10CqiRxed.end())
    //     {
    //         wbCqi = (*itCqi).second;
    //     }
    //     else
    //     {
    //         wbCqi = 1; // lowest value for trying a transmission
    //     }

    //     if (wbCqi != 0)
    //     {
    //         // CQI == 0 means "out of range" (see table 7.2.3-1 of 36.213)
    //         if (LcActivePerFlow(*it) > 0)
    //         {
    //             // this UE has data to transmit
    //             double achievableRate = 0.0;
    //             for (uint8_t k = 0; k < nLayer; k++)
    //             {
    //                 uint8_t mcs = 0;
    //                 mcs = m_amc->GetMcsFromCqi(wbCqi);
    //                 achievableRate +=
    //                     ((m_amc->GetUlTbSizeFromMcs(mcs, rbgSize) / 8) / 0.001); // = TB size / TTI

    //                 NS_LOG_DEBUG(this << " RNTI " << (*it) << " MCS " << (uint32_t)mcs
    //                                   << " achievableRate " << achievableRate);
    //             }

    //             double metric = achievableRate;

    //             if (metric > metricMax)
    //             {
    //                 metricMax = metric;
    //                 itMax = it;
    //             }
    //         } // LcActivePerFlow

    //     } // cqi

    // } // end for m_flowStatsUl

    // if (itMax == m_flowStatsUl.end())
    // {
    //     // no UE available for uplink
    //     NS_LOG_INFO(this << " any UE found");
    // }
    // else
    // {
    //     // assign all free RBGs to this UE
    //     std::vector<uint16_t> tempMap;
    //     for (int i = 0; i < rbgNum; i++)
    //     {
    //         NS_LOG_INFO(this << " ALLOCATION for RBG " << i << " of " << rbgNum);
    //         NS_LOG_DEBUG(this << " ALLOCATION for RBG " << i << " of " << rbgNum);
    //         if (rbMap.at(i) == false)
    //         {
    //             rbMap.at(i) = true;
    //             rbgAllocationMap.at(i) = i;
    //             // tempMap.push_back(i);
    //         } // end for RBG free

    //     } // end for RBGs
    //     if (rbgAllocationMap.size() > 0)
    //     {
    //         m_allocationMaps.insert(std::pair<uint16_t, std::vector<uint16_t>>((*itMax), rbgAllocationMap));
    //     }
    // }

    // // generate the transmission opportunities by grouping the RBGs of the same RNTI and
    // // creating the correspondent DCIs
    // std::map<uint16_t, std::vector<uint16_t>>::iterator itMap = m_allocationMaps.begin();
    // while (itMap != m_allocationMaps.end())
    // {
    //     // create new BuildDataListElement_s for this LC
    //     // BuildDataListElement_s newEl;
    //     // newEl.m_rnti = (*itMap).first;
    //     // create the DlDciListElement_s
    //     UlDciListElement_s newDci;
    //     newDci.m_rnti = (*itMap).first;
    //     newDci.m_rbLen = rbgSize;
    //     // newDci.m_harqProcess = UpdateHarqProcessId((*itMap).first);

    //     uint16_t lcActives = LcActivePerFlow((*itMap).first);
    //     NS_LOG_INFO(this << "Allocate user " << newDci.m_rnti << " rbg " << lcActives);
    //     if (lcActives == 0)
    //     {
    //         // Set to max value, to avoid divide by 0 below
    //         lcActives = (uint16_t)65535; // UINT16_MAX;
    //     }
    //     // uint16_t RgbPerRnti = (*itMap).second.size();
    //     std::map<uint16_t, uint8_t>::iterator itCqi;
    //     itCqi = m_p10CqiRxed.find((*itMap).first);
    //     std::map<uint16_t, uint8_t>::iterator itTxMode;
    //     itTxMode = m_uesTxMode.find((*itMap).first);
    //     if (itTxMode == m_uesTxMode.end())
    //     {
    //         NS_FATAL_ERROR("No Transmission Mode info on user " << (*itMap).first);
    //     }
    //     auto nLayer = TransmissionModesLayers::TxMode2LayerNum((*itTxMode).second);
    //     for (uint8_t j = 0; j < nLayer; j++)
    //     {
    //         if (itCqi == m_p10CqiRxed.end())
    //         {
    //             newDci.m_mcs = 0; // no info on this user -> lowest MCS
    //         }
    //         else
    //         {
    //             newDci.m_mcs = m_amc->GetMcsFromCqi((*itCqi).second);
    //         }

    //         newDci.m_tbSize = (m_amc->GetUlTbSizeFromMcs(newDci.m_mcs, rbgSize) / 8); // (size of TB in bytes according to table 7.1.7.2.1-1 of 36.213)
    //     }

    //     // std::map<uint16_t, std::vector<double>>::iterator itCqi = m_ueCqi.find((it));
    //     // int cqi = 0;
    //     // if (itCqi == m_ueCqi.end())
    //     // {
    //     //     // no cqi info about this UE
    //     //     uldci.m_mcs = 0; // MCS 0 -> UL-AMC TBD
    //     // }
    //     // else
    //     // {
    //     //     // take the lowest CQI value (worst RB)
    //     //     NS_ABORT_MSG_IF((*itCqi).second.size() == 0,
    //     //                     "CQI of RNTI = " << (*it).first << " has expired");
    //     //     double minSinr = (*itCqi).second.at(newDci.m_rbStart);
    //     //     if (minSinr == NO_SINR)
    //     //     {
    //     //         minSinr = EstimateUlSinr((*it).first, uldci.m_rbStart);
    //     //     }
    //     //     for (uint16_t i = uldci.m_rbStart; i < uldci.m_rbStart + uldci.m_rbLen; i++)
    //     //     {
    //     //         double sinr = (*itCqi).second.at(i);
    //     //         if (sinr == NO_SINR)
    //     //         {
    //     //             sinr = EstimateUlSinr((*it).first, i);
    //     //         }
    //     //         if (sinr < minSinr)
    //     //         {
    //     //             minSinr = sinr;
    //     //         }
    //     //     }

    //     //     // translate SINR -> cqi: WILD ACK: same as DL
    //     //     double s = log2(1 + (std::pow(10, minSinr / 10) / ((-std::log(5.0 * 0.00005)) / 1.5)));
    //     //     cqi = m_amc->GetCqiFromSpectralEfficiency(s);
    //     //     if (cqi == 0)
    //     //     {
    //     //         it++;
    //     //         if (it == m_ceBsrRxed.end())
    //     //         {
    //     //             // restart from the first
    //     //             it = m_ceBsrRxed.begin();
    //     //         }
    //     //         NS_LOG_DEBUG(this << " UE discarded for CQI = 0, RNTI " << uldci.m_rnti);
    //     //         // remove UE from allocation map
    //     //         for (uint16_t i = uldci.m_rbStart; i < uldci.m_rbStart + uldci.m_rbLen; i++)
    //     //         {
    //     //             rbgAllocationMap.at(i) = 0;
    //     //         }
    //     //         continue; // CQI == 0 means "out of range" (see table 7.2.3-1 of 36.213)
    //     //     }
    //     //     uldci.m_mcs = m_amc->GetMcsFromCqi(cqi);
    //     // }

    //     // std::map<LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator
    //     //     itBufReq;
    //     // for (itBufReq = m_rlcBufferReq.begin(); itBufReq != m_rlcBufferReq.end(); itBufReq++)
    //     // {
    //     //     if (((*itBufReq).first.m_rnti == (*itMap).first) &&
    //     //         (((*itBufReq).second.m_rlcTransmissionQueueSize > 0) ||
    //     //          ((*itBufReq).second.m_rlcRetransmissionQueueSize > 0) ||
    //     //          ((*itBufReq).second.m_rlcStatusPduSize > 0)))
    //     //     {
    //     //         std::vector<struct RlcPduListElement_s> newRlcPduLe;
    //     //         for (uint8_t j = 0; j < nLayer; j++)
    //     //         {
    //     //             RlcPduListElement_s newRlcEl;
    //     //             newRlcEl.m_logicalChannelIdentity = (*itBufReq).first.m_lcId;
    //     //             newRlcEl.m_size = newDci.m_tbSize / lcActives;
    //     //             NS_LOG_INFO(this << " LCID " << (uint32_t)newRlcEl.m_logicalChannelIdentity
    //     //                              << " size " << newRlcEl.m_size << " layer " << (uint16_t)j);
    //     //             newRlcPduLe.push_back(newRlcEl);
    //     //             UpdateDlRlcBufferInfo(newDci.m_rnti,
    //     //                                   newRlcEl.m_logicalChannelIdentity,
    //     //                                   newRlcEl.m_size);
    //     //         }
    //     //     }
    //     // }

    //     UpdateUlRlcBufferInfo(newDci.m_rnti, newDci.m_tbSize);
    //     newDci.m_ndi = 1;
    //     newDci.m_cceIndex = 0;
    //     newDci.m_aggrLevel = 1;
    //     newDci.m_ueTxAntennaSelection = 3; // antenna selection OFF
    //     newDci.m_hopping = false;
    //     newDci.m_n2Dmrs = 0;
    //     newDci.m_tpc = 0;            // no power control
    //     newDci.m_cqiRequest = false; // only period CQI at this stage
    //     newDci.m_ulIndex = 0;        // TDD parameter
    //     newDci.m_dai = 1;            // TDD parameter
    //     newDci.m_freqHopping = 0;
    //     newDci.m_pdcchPowerOffset = 0; // not used
    //     ret.m_dciList.push_back(newDci);

    //     // newDci.m_resAlloc = 0; // only allocation type 0 at this stage
    //     // newDci.m_rbBitmap = 0; // TBD (32 bit bitmap see 7.1.6 of 36.213)
    //     // uint32_t rbgMask = 0;
    //     // for (std::size_t k = 0; k < (*itMap).second.size(); k++)
    //     // {
    //     //     rbgMask = rbgMask + (0x1 << (*itMap).second.at(k));
    //     //     NS_LOG_INFO(this << " Allocated RBG " << (*itMap).second.at(k));
    //     // }
    //     // newDci.m_rbBitmap = rbgMask; // (32 bit bitmap see 7.1.6 of 36.213)

    //     // create the rlc PDUs -> equally divide resources among actives LCs
    //     // std::map<LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator
    //     //     itBufReq;
    //     // for (itBufReq = m_rlcBufferReq.begin(); itBufReq != m_rlcBufferReq.end(); itBufReq++)
    //     // {
    //     //     if (((*itBufReq).first.m_rnti == (*itMap).first) &&
    //     //         (((*itBufReq).second.m_rlcTransmissionQueueSize > 0) ||
    //     //          ((*itBufReq).second.m_rlcRetransmissionQueueSize > 0) ||
    //     //          ((*itBufReq).second.m_rlcStatusPduSize > 0)))
    //     //     {
    //     //         std::vector<struct RlcPduListElement_s> newRlcPduLe;
    //     //         for (uint8_t j = 0; j < nLayer; j++)
    //     //         {
    //     //             RlcPduListElement_s newRlcEl;
    //     //             newRlcEl.m_logicalChannelIdentity = (*itBufReq).first.m_lcId;
    //     //             newRlcEl.m_size = newDci.m_tbsSize.at(j) / lcActives;
    //     //             NS_LOG_INFO(this << " LCID " << (uint32_t)newRlcEl.m_logicalChannelIdentity
    //     //                              << " size " << newRlcEl.m_size << " layer " << (uint16_t)j);
    //     //             newRlcPduLe.push_back(newRlcEl);
    //     //             UpdateDlRlcBufferInfo(newDci.m_rnti,
    //     //                                   newRlcEl.m_logicalChannelIdentity,
    //     //                                   newRlcEl.m_size);
    //     //             if (m_harqOn == true)
    //     //             {
    //     //                 // store RLC PDU list for HARQ
    //     //                 std::map<uint16_t, DlHarqRlcPduListBuffer_t>::iterator itRlcPdu =
    //     //                     m_dlHarqProcessesRlcPduListBuffer.find((*itMap).first);
    //     //                 if (itRlcPdu == m_dlHarqProcessesRlcPduListBuffer.end())
    //     //                 {
    //     //                     NS_FATAL_ERROR("Unable to find RlcPdcList in HARQ buffer for RNTI "
    //     //                                    << (*itMap).first);
    //     //                 }
    //     //                 (*itRlcPdu).second.at(j).at(newDci.m_harqProcess).push_back(newRlcEl);
    //     //             }
    //     //         }
    //     //         newEl.m_rlcPduList.push_back(newRlcPduLe);
    //     //     }
    //     //     if ((*itBufReq).first.m_rnti > (*itMap).first)
    //     //     {
    //     //         break;
    //     //     }
    //     // }
    //     // for (uint8_t j = 0; j < nLayer; j++)
    //     // {
    //     //     newDci.m_ndi.push_back(1);
    //     //     newDci.m_rv.push_back(0);
    //     // }

    //     // newDci.m_tpc = 1; // 1 is mapped to 0 in Accumulated Mode and to -1 in Absolute Mode

    //     // newEl.m_dci = newDci;

    //     uint8_t harqId = 0;
    //     if (m_harqOn == true)
    //     {
    //         // store DCI for HARQ
    //         std::map<uint16_t, uint8_t>::iterator itProcId;
    //         itProcId = m_ulHarqCurrentProcessId.find(newDci.m_rnti);
    //         // std::map<uint16_t, DlHarqProcessesDciBuffer_t>::iterator itDci =
    //         //     m_dlHarqProcessesDciBuffer.find(newEl.m_rnti);
    //         if (itProcId == m_ulHarqCurrentProcessId.end())
    //         {
    //             NS_FATAL_ERROR("No info find in HARQ buffer for UE " << newDci.m_rnti);
    //         }
    //         harqId = (*itProcId).second;
    //         // (*itDci).second.at(newDci.m_harqProcess) = newDci;
    //         // refresh timer
    //         std::map<uint16_t, UlHarqProcessesDciBuffer_t>::iterator itDci =
    //             m_ulHarqProcessesDciBuffer.find(newDci.m_rnti);
    //         // std::map<uint16_t, DlHarqProcessesTimer_t>::iterator itHarqTimer =
    //         //     m_dlHarqProcessesTimer.find(newEl.m_rnti);
    //         if (itDci == m_ulHarqProcessesDciBuffer.end())
    //         {
    //             NS_FATAL_ERROR("Unable to find RNTI entry in UL DCI HARQ buffer for RNTI "
    //                            << newDci.m_rnti);
    //         }
    //         // if (itHarqTimer == m_dlHarqProcessesTimer.end())
    //         // {
    //         //     NS_FATAL_ERROR("Unable to find HARQ timer for RNTI " << (uint16_t)newEl.m_rnti);
    //         // }
    //         // (*itHarqTimer).second.at(newDci.m_harqProcess) = 0;
    //         (*itDci).second.at(harqId) = newDci;
    //         // Update HARQ process status (RV 0)
    //         std::map<uint16_t, UlHarqProcessesStatus_t>::iterator itStat =
    //             m_ulHarqProcessesStatus.find(newDci.m_rnti);
    //         if (itStat == m_ulHarqProcessesStatus.end())
    //         {
    //             NS_LOG_ERROR("No info find in HARQ buffer for UE (might change eNB) "
    //                          << newDci.m_rnti);
    //         }
    //         (*itStat).second.at(harqId) = 0;
    //     }

    //     // ...more parameters -> ignored in this version

    //     NS_LOG_INFO(this << " UE Allocation RNTI " << (*it) << " startPRB "
    //                      << (uint32_t)newDci.m_rbStart << " nPRB " << (uint32_t)newDci.m_rbLen
    //                      << " CQI " << 0 << " MCS " << (uint32_t)newDci.m_mcs << " TBsize "
    //                      << newDci.m_tbSize << " RbAlloc " << 0 << " harqId "
    //                      << (uint16_t)harqId);

    //     // ret.m_dciList.push_back(newDci);

    //     itMap++;
    // }

    std::cout << "______________________________" << std::endl << std::endl;

    m_schedSapUser->SchedUlConfigInd(ret);


    // Код для планировщика Td Maximum Throughput для восходящего канала, который был в ns3
    
    // do
    // {
    //     std::set<uint16_t>::iterator itRnti = rntiAllocated.find((*it).first);
    //     if ((itRnti != rntiAllocated.end()) || ((*it).second == 0))
    //     {
    //         // UE already allocated for UL-HARQ -> skip it
    //         NS_LOG_DEBUG(this << " UE already allocated in HARQ -> discared, RNTI " << (*it).first);
    //         it++;
    //         if (it == m_ceBsrRxed.end())
    //         {
    //             // restart from the first
    //             it = m_ceBsrRxed.begin();
    //         }
    //         continue;
    //     }
    //     if (rbAllocated + rbPerFlow - 1 > m_cschedCellConfig.m_ulBandwidth)
    //     {
    //         // limit to physical resources last resource assignment
    //         rbPerFlow = m_cschedCellConfig.m_ulBandwidth - rbAllocated;
    //         // at least 3 rbg per flow to ensure TxOpportunity >= 7 bytes
    //         if (rbPerFlow < 3)
    //         {
    //             // terminate allocation
    //             rbPerFlow = 0;
    //         }
    //     }

    //     UlDciListElement_s uldci;
    //     uldci.m_rnti = (*it).first;
    //     uldci.m_rbLen = rbPerFlow;
    //     bool allocated = false;
    //     NS_LOG_INFO(this << " RB Allocated " << rbAllocated << " rbPerFlow " << rbPerFlow
    //                      << " flows " << nflows);

        
    //     while ((!allocated) && ((rbAllocated + rbPerFlow - m_cschedCellConfig.m_ulBandwidth) < 1) &&
    //            (rbPerFlow != 0))
    //     {
    //         // check availability
    //         bool free = true;
    //         for (int j = rbAllocated; j < rbAllocated + rbPerFlow; j++)
    //         {
    //             if (rbMap.at(j) == true)
    //             {
    //                 free = false;
    //                 break;
    //             }
    //         }
    //         if (free)
    //         {
    //             uldci.m_rbStart = rbAllocated;

    //             for (int j = rbAllocated; j < rbAllocated + rbPerFlow; j++)
    //             {
    //                 rbMap.at(j) = true;
    //                 // store info on allocation for managing ul-cqi interpretation
    //                 rbgAllocationMap.at(j) = (*it).first;
    //             }
    //             rbAllocated += rbPerFlow;
    //             allocated = true;
    //             break;
    //         }
    //         rbAllocated++;
    //         if (rbAllocated + rbPerFlow - 1 > m_cschedCellConfig.m_ulBandwidth)
    //         {
    //             // limit to physical resources last resource assignment
    //             rbPerFlow = m_cschedCellConfig.m_ulBandwidth - rbAllocated;
    //             // at least 3 rbg per flow to ensure TxOpportunity >= 7 bytes
    //             if (rbPerFlow < 3)
    //             {
    //                 // terminate allocation
    //                 rbPerFlow = 0;
    //             }
    //         }
    //     }
    //     if (!allocated)
    //     {
    //         // unable to allocate new resource: finish scheduling
    //         m_nextRntiUl = (*it).first;
    //         if (ret.m_dciList.size() > 0)
    //         {
    //             m_schedSapUser->SchedUlConfigInd(ret);
    //         }
    //         m_allocationMaps.insert(
    //             std::pair<uint16_t, std::vector<uint16_t>>(params.m_sfnSf, rbgAllocationMap));
    //         return;
    //     }

    //     std::map<uint16_t, std::vector<double>>::iterator itCqi = m_ueCqi.find((*it).first);
    //     int cqi = 0;
    //     if (itCqi == m_ueCqi.end())
    //     {
    //         // no cqi info about this UE
    //         uldci.m_mcs = 0; // MCS 0 -> UL-AMC TBD
    //     }
    //     else
    //     {
    //         // take the lowest CQI value (worst RB)
    //         NS_ABORT_MSG_IF((*itCqi).second.size() == 0,
    //                         "CQI of RNTI = " << (*it).first << " has expired");
    //         double minSinr = (*itCqi).second.at(uldci.m_rbStart);
    //         if (minSinr == NO_SINR)
    //         {
    //             minSinr = EstimateUlSinr((*it).first, uldci.m_rbStart);
    //         }
    //         for (uint16_t i = uldci.m_rbStart; i < uldci.m_rbStart + uldci.m_rbLen; i++)
    //         {
    //             double sinr = (*itCqi).second.at(i);
    //             if (sinr == NO_SINR)
    //             {
    //                 sinr = EstimateUlSinr((*it).first, i);
    //             }
    //             if (sinr < minSinr)
    //             {
    //                 minSinr = sinr;
    //             }
    //         }

    //         // translate SINR -> cqi: WILD ACK: same as DL
    //         double s = log2(1 + (std::pow(10, minSinr / 10) / ((-std::log(5.0 * 0.00005)) / 1.5)));
    //         cqi = m_amc->GetCqiFromSpectralEfficiency(s);
    //         if (cqi == 0)
    //         {
    //             it++;
    //             if (it == m_ceBsrRxed.end())
    //             {
    //                 // restart from the first
    //                 it = m_ceBsrRxed.begin();
    //             }
    //             NS_LOG_DEBUG(this << " UE discarded for CQI = 0, RNTI " << uldci.m_rnti);
    //             // remove UE from allocation map
    //             for (uint16_t i = uldci.m_rbStart; i < uldci.m_rbStart + uldci.m_rbLen; i++)
    //             {
    //                 rbgAllocationMap.at(i) = 0;
    //             }
    //             continue; // CQI == 0 means "out of range" (see table 7.2.3-1 of 36.213)
    //         }
    //         uldci.m_mcs = m_amc->GetMcsFromCqi(cqi);
    //     }

    //     uldci.m_tbSize = (m_amc->GetUlTbSizeFromMcs(uldci.m_mcs, rbPerFlow) / 8);
    //     UpdateUlRlcBufferInfo(uldci.m_rnti, uldci.m_tbSize);
    //     uldci.m_ndi = 1;
    //     uldci.m_cceIndex = 0;
    //     uldci.m_aggrLevel = 1;
    //     uldci.m_ueTxAntennaSelection = 3; // antenna selection OFF
    //     uldci.m_hopping = false;
    //     uldci.m_n2Dmrs = 0;
    //     uldci.m_tpc = 0;            // no power control
    //     uldci.m_cqiRequest = false; // only period CQI at this stage
    //     uldci.m_ulIndex = 0;        // TDD parameter
    //     uldci.m_dai = 1;            // TDD parameter
    //     uldci.m_freqHopping = 0;
    //     uldci.m_pdcchPowerOffset = 0; // not used
    //     ret.m_dciList.push_back(uldci);
    //     // store DCI for HARQ_PERIOD
    //     uint8_t harqId = 0;
    //     if (m_harqOn == true)
    //     {
    //         std::map<uint16_t, uint8_t>::iterator itProcId;
    //         itProcId = m_ulHarqCurrentProcessId.find(uldci.m_rnti);
    //         if (itProcId == m_ulHarqCurrentProcessId.end())
    //         {
    //             NS_FATAL_ERROR("No info find in HARQ buffer for UE " << uldci.m_rnti);
    //         }
    //         harqId = (*itProcId).second;
    //         std::map<uint16_t, UlHarqProcessesDciBuffer_t>::iterator itDci =
    //             m_ulHarqProcessesDciBuffer.find(uldci.m_rnti);
    //         if (itDci == m_ulHarqProcessesDciBuffer.end())
    //         {
    //             NS_FATAL_ERROR("Unable to find RNTI entry in UL DCI HARQ buffer for RNTI "
    //                            << uldci.m_rnti);
    //         }
    //         (*itDci).second.at(harqId) = uldci;
    //         // Update HARQ process status (RV 0)
    //         std::map<uint16_t, UlHarqProcessesStatus_t>::iterator itStat =
    //             m_ulHarqProcessesStatus.find(uldci.m_rnti);
    //         if (itStat == m_ulHarqProcessesStatus.end())
    //         {
    //             NS_LOG_ERROR("No info find in HARQ buffer for UE (might change eNB) "
    //                          << uldci.m_rnti);
    //         }
    //         (*itStat).second.at(harqId) = 0;
    //     }

    //     NS_LOG_INFO(this << " UE Allocation RNTI " << (*it).first << " startPRB "
    //                      << (uint32_t)uldci.m_rbStart << " nPRB " << (uint32_t)uldci.m_rbLen
    //                      << " CQI " << cqi << " MCS " << (uint32_t)uldci.m_mcs << " TBsize "
    //                      << uldci.m_tbSize << " RbAlloc " << rbAllocated << " harqId "
    //                      << (uint16_t)harqId);

    //     std::cout << "RNTI: " << (*it).first << std::endl;
    //     std::cout << "RBAllocated: " << rbAllocated << std::endl;
    //     std::cout << "RBPerFlow: " << rbPerFlow << std::endl;
    //     it++;
    //     if (it == m_ceBsrRxed.end())
    //     {
    //         // restart from the first
    //         it = m_ceBsrRxed.begin();
    //     }
    //     if ((rbAllocated == m_cschedCellConfig.m_ulBandwidth) || (rbPerFlow == 0))
    //     {
    //         // Stop allocation: no more PRBs
    //         m_nextRntiUl = (*it).first;
    //         break;
    //     }
    // } while (((*it).first != m_nextRntiUl) && (rbPerFlow != 0));

    // m_allocationMaps.insert(
    //     std::pair<uint16_t, std::vector<uint16_t>>(params.m_sfnSf, rbgAllocationMap));
    // m_schedSapUser->SchedUlConfigInd(ret);



    
    // NS_LOG_FUNCTION(this << " UL - Frame no. " << (params.m_sfnSf >> 4) << " subframe no. "
    //                      << (0xF & params.m_sfnSf) << " size " << params.m_ulInfoList.size());
    // // API generated by RLC for triggering the scheduling of a DL subframe

    // // evaluate the relative channel quality indicator for each UE per each RBG
    // // (since we are using allocation type 0 the small unit of allocation is RBG)
    // // Resource allocation type 0 (see sec 7.1.6.1 of 36.213)

    // RefreshUlCqiMaps();

    // int rbgSize = GetRbgSize(m_cschedCellConfig.m_ulBandwidth);
    // int rbgNum = m_cschedCellConfig.m_dlBandwidth / rbgSize;
    // std::map<uint16_t, std::vector<uint16_t>> allocationMap; // RBs map per RNTI
    // std::vector<bool> rbgMap;                                // global RBGs map
    // uint16_t rbgAllocatedNum = 0;
    // std::set<uint16_t> rntiAllocated;
    // rbgMap.resize(m_cschedCellConfig.m_ulBandwidth / rbgSize, false);
    // FfMacSchedSapUser::SchedUlConfigIndParameters ret;

    // //   update UL HARQ proc id
    // std::map<uint16_t, uint8_t>::iterator itProcId;
    // for (itProcId = m_ulHarqCurrentProcessId.begin(); itProcId != m_ulHarqCurrentProcessId.end();
    //      itProcId++)
    // {
    //     (*itProcId).second = ((*itProcId).second + 1) % HARQ_PROC_NUM;
    // }

    // // RACH Allocation
    // m_rachAllocationMap.resize(m_cschedCellConfig.m_ulBandwidth, 0);
    // uint16_t rbStart = 0;
    // std::vector<struct RachListElement_s>::iterator itRach;
    // for (itRach = m_rachList.begin(); itRach != m_rachList.end(); itRach++)
    // {
    //     NS_ASSERT_MSG(m_amc->GetUlTbSizeFromMcs(m_ulGrantMcs, m_cschedCellConfig.m_ulBandwidth) >
    //                       (*itRach).m_estimatedSize,
    //                   " Default UL Grant MCS does not allow to send RACH messages");
    //     BuildRarListElement_s newRar;
    //     newRar.m_rnti = (*itRach).m_rnti;
    //     // DL-RACH Allocation
    //     // Ideal: no needs of configuring m_dci
    //     // UL-RACH Allocation
    //     newRar.m_grant.m_rnti = newRar.m_rnti;
    //     newRar.m_grant.m_mcs = m_ulGrantMcs;
    //     uint16_t rbLen = 1;
    //     uint16_t tbSizeBits = 0;
    //     // find lowest TB size that fits UL grant estimated size
    //     while ((tbSizeBits < (*itRach).m_estimatedSize) &&
    //            (rbStart + rbLen < m_cschedCellConfig.m_ulBandwidth))
    //     {
    //         rbLen++;
    //         tbSizeBits = m_amc->GetUlTbSizeFromMcs(m_ulGrantMcs, rbLen);
    //     }
    //     if (tbSizeBits < (*itRach).m_estimatedSize)
    //     {
    //         // no more allocation space: finish allocation
    //         break;
    //     }
    //     newRar.m_grant.m_rbStart = rbStart;
    //     newRar.m_grant.m_rbLen = rbLen;
    //     newRar.m_grant.m_tbSize = tbSizeBits / 8;
    //     newRar.m_grant.m_hopping = false;
    //     newRar.m_grant.m_tpc = 0;
    //     newRar.m_grant.m_cqiRequest = false;
    //     newRar.m_grant.m_ulDelay = false;
    //     NS_LOG_INFO(this << " UL grant allocated to RNTI " << (*itRach).m_rnti << " rbStart "
    //                      << rbStart << " rbLen " << rbLen << " MCS " << m_ulGrantMcs << " tbSize "
    //                      << newRar.m_grant.m_tbSize);
    //     for (uint16_t i = rbStart; i < rbStart + rbLen; i++)
    //     {
    //         m_rachAllocationMap.at(i) = (*itRach).m_rnti;
    //     }

    //     if (m_harqOn == true)
    //     {
    //         // generate UL-DCI for HARQ retransmissions
    //         UlDciListElement_s uldci;
    //         uldci.m_rnti = newRar.m_rnti;
    //         uldci.m_rbLen = rbLen;
    //         uldci.m_rbStart = rbStart;
    //         uldci.m_mcs = m_ulGrantMcs;
    //         uldci.m_tbSize = tbSizeBits / 8;
    //         uldci.m_ndi = 1;
    //         uldci.m_cceIndex = 0;
    //         uldci.m_aggrLevel = 1;
    //         uldci.m_ueTxAntennaSelection = 3; // antenna selection OFF
    //         uldci.m_hopping = false;
    //         uldci.m_n2Dmrs = 0;
    //         uldci.m_tpc = 0;            // no power control
    //         uldci.m_cqiRequest = false; // only period CQI at this stage
    //         uldci.m_ulIndex = 0;        // TDD parameter
    //         uldci.m_dai = 1;            // TDD parameter
    //         uldci.m_freqHopping = 0;
    //         uldci.m_pdcchPowerOffset = 0; // not used

    //         uint8_t harqId = 0;
    //         std::map<uint16_t, uint8_t>::iterator itProcId;
    //         itProcId = m_ulHarqCurrentProcessId.find(uldci.m_rnti);
    //         if (itProcId == m_ulHarqCurrentProcessId.end())
    //         {
    //             NS_FATAL_ERROR("No info find in HARQ buffer for UE " << uldci.m_rnti);
    //         }
    //         harqId = (*itProcId).second;
    //         std::map<uint16_t, UlHarqProcessesDciBuffer_t>::iterator itDci =
    //             m_ulHarqProcessesDciBuffer.find(uldci.m_rnti);
    //         if (itDci == m_ulHarqProcessesDciBuffer.end())
    //         {
    //             NS_FATAL_ERROR("Unable to find RNTI entry in UL DCI HARQ buffer for RNTI "
    //                            << uldci.m_rnti);
    //         }
    //         (*itDci).second.at(harqId) = uldci;
    //     }

    //     rbStart = rbStart + rbLen;
    //     ret.m_buildRarList.push_back(newRar);
    // }
    // m_rachList.clear();

    // // Process DL HARQ feedback
    // RefreshHarqProcesses();
    // // retrieve past HARQ retx buffered
    // if (m_ulInfoListBuffered.size() > 0)
    // {
    //     if (params.m_ulInfoList.size() > 0)
    //     {
    //         NS_LOG_INFO(this << " Received DL-HARQ feedback");
    //         m_ulInfoListBuffered.insert(m_ulInfoListBuffered.end(),
    //                                     params.m_ulInfoList.begin(),
    //                                     params.m_ulInfoList.end());
    //     }
    // }
    // else
    // {
    //     if (params.m_ulInfoList.size() > 0)
    //     {
    //         m_ulInfoListBuffered = params.m_ulInfoList;
    //     }
    // }
    // if (m_harqOn == false)
    // {
    //     // Ignore HARQ feedback
    //     m_ulInfoListBuffered.clear();
    // }
    // std::vector<struct UlInfoListElement_s> ulInfoListUntxed;
    // for (std::size_t i = 0; i < m_ulInfoListBuffered.size(); i++)
    // {
    //     std::set<uint16_t>::iterator itRnti = rntiAllocated.find(m_ulInfoListBuffered.at(i).m_rnti);
    //     if (itRnti != rntiAllocated.end())
    //     {
    //         // RNTI already allocated for retx
    //         continue;
    //     }
    //     auto nLayers = m_ulInfoListBuffered.at(i).m_harqStatus.size();
    //     std::vector<bool> retx;
    //     NS_LOG_INFO(this << " Processing DLHARQ feedback");
    //     if (nLayers == 1)
    //     {
    //         retx.push_back(m_ulInfoListBuffered.at(i).m_harqStatus.at(0) ==
    //                        UlInfoListElement_s::NACK);
    //         retx.push_back(false);
    //     }
    //     else
    //     {
    //         retx.push_back(m_ulInfoListBuffered.at(i).m_harqStatus.at(0) ==
    //                        UlInfoListElement_s::NACK);
    //         retx.push_back(m_ulInfoListBuffered.at(i).m_harqStatus.at(1) ==
    //                        UlInfoListElement_s::NACK);
    //     }
    //     if (retx.at(0) || retx.at(1))
    //     {
    //         // retrieve HARQ process information
    //         uint16_t rnti = m_ulInfoListBuffered.at(i).m_rnti;
    //         uint8_t harqId = m_ulInfoListBuffered.at(i).m_harqProcessId;
    //         NS_LOG_INFO(this << " HARQ retx RNTI " << rnti << " harqId " << (uint16_t)harqId);
    //         std::map<uint16_t, UlHarqProcessesDciBuffer_t>::iterator itHarq =
    //             m_ulHarqProcessesDciBuffer.find(rnti);
    //         if (itHarq == m_ulHarqProcessesDciBuffer.end())
    //         {
    //             NS_FATAL_ERROR("No info find in HARQ buffer for UE " << rnti);
    //         }

    //         UlDciListElement_s dci = (*itHarq).second.at(harqId);
    //         int rv = 0;
    //         if (dci.m_rv.size() == 1)
    //         {
    //             rv = dci.m_rv.at(0);
    //         }
    //         else
    //         {
    //             rv = (dci.m_rv.at(0) > dci.m_rv.at(1) ? dci.m_rv.at(0) : dci.m_rv.at(1));
    //         }

    //         if (rv == 3)
    //         {
    //             // maximum number of retx reached -> drop process
    //             NS_LOG_INFO("Maximum number of retransmissions reached -> drop process");
    //             std::map<uint16_t, UlHarqProcessesStatus_t>::iterator it =
    //                 m_ulHarqProcessesStatus.find(rnti);
    //             if (it == m_ulHarqProcessesStatus.end())
    //             {
    //                 NS_LOG_ERROR("No info find in HARQ buffer for UE (might change eNB) "
    //                              << m_ulInfoListBuffered.at(i).m_rnti);
    //             }
    //             (*it).second.at(harqId) = 0;
    //             std::map<uint16_t, UlHarqRlcPduListBuffer_t>::iterator itRlcPdu =
    //                 m_ulHarqProcessesRlcPduListBuffer.find(rnti);
    //             if (itRlcPdu == m_ulHarqProcessesRlcPduListBuffer.end())
    //             {
    //                 NS_FATAL_ERROR("Unable to find RlcPdcList in HARQ buffer for RNTI "
    //                                << m_ulInfoListBuffered.at(i).m_rnti);
    //             }
    //             for (std::size_t k = 0; k < (*itRlcPdu).second.size(); k++)
    //             {
    //                 (*itRlcPdu).second.at(k).at(harqId).clear();
    //             }
    //             continue;
    //         }
    //         // check the feasibility of retransmitting on the same RBGs
    //         // translate the DCI to Spectrum framework
    //         std::vector<int> dciRbg;
    //         uint32_t mask = 0x1;
    //         NS_LOG_INFO("Original RBGs " << dci.m_rbBitmap << " rnti " << dci.m_rnti);
    //         for (int j = 0; j < 32; j++)
    //         {
    //             if (((dci.m_rbBitmap & mask) >> j) == 1)
    //             {
    //                 dciRbg.push_back(j);
    //                 NS_LOG_INFO("\t" << j);
    //             }
    //             mask = (mask << 1);
    //         }
    //         bool free = true;
    //         for (std::size_t j = 0; j < dciRbg.size(); j++)
    //         {
    //             if (rbgMap.at(dciRbg.at(j)) == true)
    //             {
    //                 free = false;
    //                 break;
    //             }
    //         }
    //         if (free)
    //         {
    //             // use the same RBGs for the retx
    //             // reserve RBGs
    //             for (std::size_t j = 0; j < dciRbg.size(); j++)
    //             {
    //                 rbgMap.at(dciRbg.at(j)) = true;
    //                 NS_LOG_INFO("RBG " << dciRbg.at(j) << " assigned");
    //                 rbgAllocatedNum++;
    //             }

    //             NS_LOG_INFO(this << " Send retx in the same RBGs");
    //         }
    //         else
    //         {
    //             // find RBGs for sending HARQ retx
    //             uint8_t j = 0;
    //             uint8_t rbgId = (dciRbg.at(dciRbg.size() - 1) + 1) % rbgNum;
    //             uint8_t startRbg = dciRbg.at(dciRbg.size() - 1);
    //             std::vector<bool> rbgMapCopy = rbgMap;
    //             while ((j < dciRbg.size()) && (startRbg != rbgId))
    //             {
    //                 if (rbgMapCopy.at(rbgId) == false)
    //                 {
    //                     rbgMapCopy.at(rbgId) = true;
    //                     dciRbg.at(j) = rbgId;
    //                     j++;
    //                 }
    //                 rbgId = (rbgId + 1) % rbgNum;
    //             }
    //             if (j == dciRbg.size())
    //             {
    //                 // find new RBGs -> update DCI map
    //                 uint32_t rbgMask = 0;
    //                 for (std::size_t k = 0; k < dciRbg.size(); k++)
    //                 {
    //                     rbgMask = rbgMask + (0x1 << dciRbg.at(k));
    //                     rbgAllocatedNum++;
    //                 }
    //                 dci.m_rbBitmap = rbgMask;
    //                 rbgMap = rbgMapCopy;
    //                 NS_LOG_INFO(this << " Move retx in RBGs " << dciRbg.size());
    //             }
    //             else
    //             {
    //                 // HARQ retx cannot be performed on this TTI -> store it
    //                 dlInfoListUntxed.push_back(m_ulInfoListBuffered.at(i));
    //                 NS_LOG_INFO(this << " No resource for this retx -> buffer it");
    //             }
    //         }
    //         // retrieve RLC PDU list for retx TBsize and update DCI
    //         BuildDataListElement_s newEl;
    //         std::map<uint16_t, UlHarqRlcPduListBuffer_t>::iterator itRlcPdu =
    //             m_ulHarqProcessesRlcPduListBuffer.find(rnti);
    //         if (itRlcPdu == m_ulHarqProcessesRlcPduListBuffer.end())
    //         {
    //             NS_FATAL_ERROR("Unable to find RlcPdcList in HARQ buffer for RNTI " << rnti);
    //         }
    //         for (std::size_t j = 0; j < nLayers; j++)
    //         {
    //             if (retx.at(j))
    //             {
    //                 if (j >= dci.m_ndi.size())
    //                 {
    //                     // for avoiding errors in MIMO transient phases
    //                     dci.m_ndi.push_back(0);
    //                     dci.m_rv.push_back(0);
    //                     dci.m_mcs.push_back(0);
    //                     dci.m_tbsSize.push_back(0);
    //                     NS_LOG_INFO(this << " layer " << (uint16_t)j
    //                                      << " no txed (MIMO transition)");
    //                 }
    //                 else
    //                 {
    //                     dci.m_ndi.at(j) = 0;
    //                     dci.m_rv.at(j)++;
    //                     (*itHarq).second.at(harqId).m_rv.at(j)++;
    //                     NS_LOG_INFO(this << " layer " << (uint16_t)j << " RV "
    //                                      << (uint16_t)dci.m_rv.at(j));
    //                 }
    //             }
    //             else
    //             {
    //                 // empty TB of layer j
    //                 dci.m_ndi.at(j) = 0;
    //                 dci.m_rv.at(j) = 0;
    //                 dci.m_mcs.at(j) = 0;
    //                 dci.m_tbsSize.at(j) = 0;
    //                 NS_LOG_INFO(this << " layer " << (uint16_t)j << " no retx");
    //             }
    //         }
    //         for (std::size_t k = 0; k < (*itRlcPdu).second.at(0).at(dci.m_harqProcess).size(); k++)
    //         {
    //             std::vector<struct RlcPduListElement_s> rlcPduListPerLc;
    //             for (std::size_t j = 0; j < nLayers; j++)
    //             {
    //                 if (retx.at(j))
    //                 {
    //                     if (j < dci.m_ndi.size())
    //                     {
    //                         NS_LOG_INFO(" layer " << (uint16_t)j << " tb size "
    //                                               << dci.m_tbsSize.at(j));
    //                         rlcPduListPerLc.push_back(
    //                             (*itRlcPdu).second.at(j).at(dci.m_harqProcess).at(k));
    //                     }
    //                 }
    //                 else
    //                 { // if no retx needed on layer j, push an RlcPduListElement_s object with
    //                   // m_size=0 to keep the size of rlcPduListPerLc vector = 2 in case of MIMO
    //                     NS_LOG_INFO(" layer " << (uint16_t)j << " tb size " << dci.m_tbsSize.at(j));
    //                     RlcPduListElement_s emptyElement;
    //                     emptyElement.m_logicalChannelIdentity = (*itRlcPdu)
    //                                                                 .second.at(j)
    //                                                                 .at(dci.m_harqProcess)
    //                                                                 .at(k)
    //                                                                 .m_logicalChannelIdentity;
    //                     emptyElement.m_size = 0;
    //                     rlcPduListPerLc.push_back(emptyElement);
    //                 }
    //             }

    //             if (rlcPduListPerLc.size() > 0)
    //             {
    //                 newEl.m_rlcPduList.push_back(rlcPduListPerLc);
    //             }
    //         }
    //         newEl.m_rnti = rnti;
    //         newEl.m_dci = dci;
    //         (*itHarq).second.at(harqId).m_rv = dci.m_rv;
    //         // refresh timer
    //         std::map<uint16_t, UlHarqProcessesTimer_t>::iterator itHarqTimer =
    //             m_ulHarqProcessesTimer.find(rnti);
    //         if (itHarqTimer == m_ulHarqProcessesTimer.end())
    //         {
    //             NS_FATAL_ERROR("Unable to find HARQ timer for RNTI " << (uint16_t)rnti);
    //         }
    //         (*itHarqTimer).second.at(harqId) = 0;
    //         ret.m_buildDataList.push_back(newEl);
    //         rntiAllocated.insert(rnti);
    //     }
    //     else
    //     {
    //         // update HARQ process status
    //         NS_LOG_INFO(this << " HARQ received ACK for UE " << m_ulInfoListBuffered.at(i).m_rnti);
    //         std::map<uint16_t, DlHarqProcessesStatus_t>::iterator it =
    //             m_ulHarqProcessesStatus.find(m_ulInfoListBuffered.at(i).m_rnti);
    //         if (it == m_ulHarqProcessesStatus.end())
    //         {
    //             NS_FATAL_ERROR("No info find in HARQ buffer for UE "
    //                            << m_ulInfoListBuffered.at(i).m_rnti);
    //         }
    //         (*it).second.at(m_ulInfoListBuffered.at(i).m_harqProcessId) = 0;
    //         std::map<uint16_t, UlHarqRlcPduListBuffer_t>::iterator itRlcPdu =
    //             m_ulHarqProcessesRlcPduListBuffer.find(m_ulInfoListBuffered.at(i).m_rnti);
    //         if (itRlcPdu == m_ulHarqProcessesRlcPduListBuffer.end())
    //         {
    //             NS_FATAL_ERROR("Unable to find RlcPdcList in HARQ buffer for RNTI "
    //                            << m_ulInfoListBuffered.at(i).m_rnti);
    //         }
    //         for (std::size_t k = 0; k < (*itRlcPdu).second.size(); k++)
    //         {
    //             (*itRlcPdu).second.at(k).at(m_ulInfoListBuffered.at(i).m_harqProcessId).clear();
    //         }
    //     }
    // }
    // m_ulInfoListBuffered.clear();
    // m_ulInfoListBuffered = ulInfoListUntxed;

    // if (rbgAllocatedNum == rbgNum)
    // {
    //     // all the RBGs are already allocated -> exit
    //     if ((ret.m_buildDataList.size() > 0) || (ret.m_buildRarList.size() > 0))
    //     {
    //         m_schedSapUser->SchedUlConfigInd(ret);
    //     }
    //     return;
    // }

    // std::set<uint16_t>::iterator it;
    // std::set<uint16_t>::iterator itMax = m_flowStatsUl.end();
    // double metricMax = 0.0;
    // for (it = m_flowStatsUl.begin(); it != m_flowStatsUl.end(); it++)
    // {
    //     std::set<uint16_t>::iterator itRnti = rntiAllocated.find((*it));
    //     if ((itRnti != rntiAllocated.end()) || (!HarqProcessAvailability((*it))))
    //     {
    //         // UE already allocated for HARQ or without HARQ process available -> drop it
    //         if (itRnti != rntiAllocated.end())
    //         {
    //             NS_LOG_DEBUG(this << " RNTI discared for HARQ tx" << (uint16_t)(*it));
    //         }
    //         if (!HarqProcessAvailability((*it)))
    //         {
    //             NS_LOG_DEBUG(this << " RNTI discared for HARQ id" << (uint16_t)(*it));
    //         }

    //         continue;
    //     }

    //     std::map<uint16_t, uint8_t>::iterator itTxMode;
    //     itTxMode = m_uesTxMode.find((*it));
    //     if (itTxMode == m_uesTxMode.end())
    //     {
    //         NS_FATAL_ERROR("No Transmission Mode info on user " << (*it));
    //     }
    //     auto nLayer = TransmissionModesLayers::TxMode2LayerNum((*itTxMode).second);
    //     std::map<uint16_t, uint8_t>::iterator itCqi = m_p10CqiRxed.find((*it));
    //     uint8_t wbCqi = 0;
    //     if (itCqi != m_p10CqiRxed.end())
    //     {
    //         wbCqi = (*itCqi).second;
    //     }
    //     else
    //     {
    //         wbCqi = 1; // lowest value for trying a transmission
    //     }

    //     if (wbCqi != 0)
    //     {
    //         // CQI == 0 means "out of range" (see table 7.2.3-1 of 36.213)
    //         if (LcActivePerFlow(*it) > 0)
    //         {
    //             // this UE has data to transmit
    //             double achievableRate = 0.0;
    //             for (uint8_t k = 0; k < nLayer; k++)
    //             {
    //                 uint8_t mcs = 0;
    //                 mcs = m_amc->GetMcsFromCqi(wbCqi);
    //                 achievableRate +=
    //                     ((m_amc->GetDlTbSizeFromMcs(mcs, rbgSize) / 8) / 0.001); // = TB size / TTI

    //                 NS_LOG_DEBUG(this << " RNTI " << (*it) << " MCS " << (uint32_t)mcs
    //                                   << " achievableRate " << achievableRate);
    //             }

    //             double metric = achievableRate;

    //             if (metric > metricMax)
    //             {
    //                 metricMax = metric;
    //                 itMax = it;
    //             }
    //         } // LcActivePerFlow

    //     } // cqi

    // } // end for m_flowStatsDl

    // if (itMax == m_flowStatsUl.end())
    // {
    //     // no UE available for downlink
    //     NS_LOG_INFO(this << " any UE found");
    // }
    // else
    // {
    //     // assign all free RBGs to this UE
    //     std::vector<uint16_t> tempMap;
    //     for (int i = 0; i < rbgNum; i++)
    //     {
    //         NS_LOG_INFO(this << " ALLOCATION for RBG " << i << " of " << rbgNum);
    //         NS_LOG_DEBUG(this << " ALLOCATION for RBG " << i << " of " << rbgNum);
    //         if (rbgMap.at(i) == false)
    //         {
    //             rbgMap.at(i) = true;
    //             tempMap.push_back(i);
    //         } // end for RBG free

    //     } // end for RBGs
    //     if (tempMap.size() > 0)
    //     {
    //         allocationMap.insert(std::pair<uint16_t, std::vector<uint16_t>>((*itMax), tempMap));
    //     }
    // }

    // // generate the transmission opportunities by grouping the RBGs of the same RNTI and
    // // creating the correspondent DCIs
    // std::map<uint16_t, std::vector<uint16_t>>::iterator itMap = allocationMap.begin();
    // while (itMap != allocationMap.end())
    // {
    //     // create new BuildDataListElement_s for this LC
    //     BuildDataListElement_s newEl;
    //     newEl.m_rnti = (*itMap).first;
    //     // create the DlDciListElement_s
    //     UlDciListElement_s newDci;
    //     newDci.m_rnti = (*itMap).first;
    //     newDci.m_harqProcess = UpdateHarqProcessId((*itMap).first);

    //     uint16_t lcActives = LcActivePerFlow((*itMap).first);
    //     NS_LOG_INFO(this << "Allocate user " << newEl.m_rnti << " rbg " << lcActives);
    //     if (lcActives == 0)
    //     {
    //         // Set to max value, to avoid divide by 0 below
    //         lcActives = (uint16_t)65535; // UINT16_MAX;
    //     }
    //     uint16_t RgbPerRnti = (*itMap).second.size();
    //     std::map<uint16_t, uint8_t>::iterator itCqi;
    //     itCqi = m_p10CqiRxed.find((*itMap).first);
    //     std::map<uint16_t, uint8_t>::iterator itTxMode;
    //     itTxMode = m_uesTxMode.find((*itMap).first);
    //     if (itTxMode == m_uesTxMode.end())
    //     {
    //         NS_FATAL_ERROR("No Transmission Mode info on user " << (*itMap).first);
    //     }
    //     auto nLayer = TransmissionModesLayers::TxMode2LayerNum((*itTxMode).second);
    //     for (uint8_t j = 0; j < nLayer; j++)
    //     {
    //         if (itCqi == m_p10CqiRxed.end())
    //         {
    //             newDci.m_mcs.push_back(0); // no info on this user -> lowest MCS
    //         }
    //         else
    //         {
    //             newDci.m_mcs.push_back(m_amc->GetMcsFromCqi((*itCqi).second));
    //         }

    //         int tbSize = (m_amc->GetDlTbSizeFromMcs(newDci.m_mcs.at(j), RgbPerRnti * rbgSize) /
    //                       8); // (size of TB in bytes according to table 7.1.7.2.1-1 of 36.213)
    //         newDci.m_tbsSize.push_back(tbSize);
    //     }

    //     newDci.m_resAlloc = 0; // only allocation type 0 at this stage
    //     newDci.m_rbBitmap = 0; // TBD (32 bit bitmap see 7.1.6 of 36.213)
    //     uint32_t rbgMask = 0;
    //     for (std::size_t k = 0; k < (*itMap).second.size(); k++)
    //     {
    //         rbgMask = rbgMask + (0x1 << (*itMap).second.at(k));
    //         NS_LOG_INFO(this << " Allocated RBG " << (*itMap).second.at(k));
    //     }
    //     newDci.m_rbBitmap = rbgMask; // (32 bit bitmap see 7.1.6 of 36.213)

    //     // create the rlc PDUs -> equally divide resources among actives LCs
    //     std::map<LteFlowId_t, FfMacSchedSapProvider::SchedUlRlcBufferReqParameters>::iterator
    //         itBufReq;
    //     for (itBufReq = m_rlcBufferReq.begin(); itBufReq != m_rlcBufferReq.end(); itBufReq++)
    //     {
    //         if (((*itBufReq).first.m_rnti == (*itMap).first) &&
    //             (((*itBufReq).second.m_rlcTransmissionQueueSize > 0) ||
    //              ((*itBufReq).second.m_rlcRetransmissionQueueSize > 0) ||
    //              ((*itBufReq).second.m_rlcStatusPduSize > 0)))
    //         {
    //             std::vector<struct RlcPduListElement_s> newRlcPduLe;
    //             for (uint8_t j = 0; j < nLayer; j++)
    //             {
    //                 RlcPduListElement_s newRlcEl;
    //                 newRlcEl.m_logicalChannelIdentity = (*itBufReq).first.m_lcId;
    //                 newRlcEl.m_size = newDci.m_tbsSize.at(j) / lcActives;
    //                 NS_LOG_INFO(this << " LCID " << (uint32_t)newRlcEl.m_logicalChannelIdentity
    //                                  << " size " << newRlcEl.m_size << " layer " << (uint16_t)j);
    //                 newRlcPduLe.push_back(newRlcEl);
    //                 UpdateDlRlcBufferInfo(newDci.m_rnti,
    //                                       newRlcEl.m_logicalChannelIdentity,
    //                                       newRlcEl.m_size);
    //                 if (m_harqOn == true)
    //                 {
    //                     // store RLC PDU list for HARQ
    //                     std::map<uint16_t, UlHarqRlcPduListBuffer_t>::iterator itRlcPdu =
    //                         m_ulHarqProcessesRlcPduListBuffer.find((*itMap).first);
    //                     if (itRlcPdu == m_ulHarqProcessesRlcPduListBuffer.end())
    //                     {
    //                         NS_FATAL_ERROR("Unable to find RlcPdcList in HARQ buffer for RNTI "
    //                                        << (*itMap).first);
    //                     }
    //                     (*itRlcPdu).second.at(j).at(newDci.m_harqProcess).push_back(newRlcEl);
    //                 }
    //             }
    //             newEl.m_rlcPduList.push_back(newRlcPduLe);
    //         }
    //         if ((*itBufReq).first.m_rnti > (*itMap).first)
    //         {
    //             break;
    //         }
    //     }
    //     for (uint8_t j = 0; j < nLayer; j++)
    //     {
    //         newDci.m_ndi.push_back(1);
    //         newDci.m_rv.push_back(0);
    //     }

    //     newDci.m_tpc = 1; // 1 is mapped to 0 in Accumulated Mode and to -1 in Absolute Mode

    //     newEl.m_dci = newDci;

    //     if (m_harqOn == true)
    //     {
    //         // store DCI for HARQ
    //         std::map<uint16_t, UlHarqProcessesDciBuffer_t>::iterator itDci =
    //             m_ulHarqProcessesDciBuffer.find(newEl.m_rnti);
    //         if (itDci == m_ulHarqProcessesDciBuffer.end())
    //         {
    //             NS_FATAL_ERROR("Unable to find RNTI entry in DCI HARQ buffer for RNTI "
    //                            << newEl.m_rnti);
    //         }
    //         (*itDci).second.at(newDci.m_harqProcess) = newDci;
    //         // refresh timer
    //         std::map<uint16_t, DlHarqProcessesTimer_t>::iterator itHarqTimer =
    //             m_ulHarqProcessesTimer.find(newEl.m_rnti);
    //         if (itHarqTimer == m_ulHarqProcessesTimer.end())
    //         {
    //             NS_FATAL_ERROR("Unable to find HARQ timer for RNTI " << (uint16_t)newEl.m_rnti);
    //         }
    //         (*itHarqTimer).second.at(newDci.m_harqProcess) = 0;
    //     }

    //     // ...more parameters -> ignored in this version

    //     ret.m_buildDataList.push_back(newEl);

    //     itMap++;
    // }                               // end while allocation
    // ret.m_nrOfPdcchOfdmSymbols = 1; /// \todo check correct value according the DCIs txed

    // m_schedSapUser->SchedUlConfigInd(ret);
}

void
TdMtFfMacScheduler::DoSchedUlNoiseInterferenceReq(
    const struct FfMacSchedSapProvider::SchedUlNoiseInterferenceReqParameters& params)
{
    NS_LOG_FUNCTION(this);
}

void
TdMtFfMacScheduler::DoSchedUlSrInfoReq(
    const struct FfMacSchedSapProvider::SchedUlSrInfoReqParameters& params)
{
    NS_LOG_FUNCTION(this);
}

void
TdMtFfMacScheduler::DoSchedUlMacCtrlInfoReq(
    const struct FfMacSchedSapProvider::SchedUlMacCtrlInfoReqParameters& params)
{
    NS_LOG_FUNCTION(this);

    std::map<uint16_t, uint32_t>::iterator it;

    for (unsigned int i = 0; i < params.m_macCeList.size(); i++)
    {
        if (params.m_macCeList.at(i).m_macCeType == MacCeListElement_s::BSR)
        {
            // buffer status report
            // note that this scheduler does not differentiate the
            // allocation according to which LCGs have more/less bytes
            // to send.
            // Hence the BSR of different LCGs are just summed up to get
            // a total queue size that is used for allocation purposes.

            uint32_t buffer = 0;
            for (uint8_t lcg = 0; lcg < 4; ++lcg)
            {
                uint8_t bsrId = params.m_macCeList.at(i).m_macCeValue.m_bufferStatus.at(lcg);
                buffer += BufferSizeLevelBsr::BsrId2BufferSize(bsrId);
            }

            uint16_t rnti = params.m_macCeList.at(i).m_rnti;
            NS_LOG_LOGIC(this << "RNTI=" << rnti << " buffer=" << buffer);
            it = m_ceBsrRxed.find(rnti);
            if (it == m_ceBsrRxed.end())
            {
                // create the new entry
                m_ceBsrRxed.insert(std::pair<uint16_t, uint32_t>(rnti, buffer));
            }
            else
            {
                // update the buffer size value
                (*it).second = buffer;
            }
        }
    }
}

void
TdMtFfMacScheduler::DoSchedUlCqiInfoReq(
    const struct FfMacSchedSapProvider::SchedUlCqiInfoReqParameters& params)
{
    NS_LOG_FUNCTION(this);
    // retrieve the allocation for this subframe
    switch (m_ulCqiFilter)
    {
    case FfMacScheduler::SRS_UL_CQI: {
        // filter all the CQIs that are not SRS based
        if (params.m_ulCqi.m_type != UlCqi_s::SRS)
        {
            return;
        }
    }
    break;
    case FfMacScheduler::PUSCH_UL_CQI: {
        // filter all the CQIs that are not SRS based
        if (params.m_ulCqi.m_type != UlCqi_s::PUSCH)
        {
            return;
        }
    }
    break;
    default:
        NS_FATAL_ERROR("Unknown UL CQI type");
    }

    switch (params.m_ulCqi.m_type)
    {
    case UlCqi_s::PUSCH: {
        std::map<uint16_t, std::vector<uint16_t>>::iterator itMap;
        std::map<uint16_t, std::vector<double>>::iterator itCqi;
        NS_LOG_DEBUG(this << " Collect PUSCH CQIs of Frame no. " << (params.m_sfnSf >> 4)
                          << " subframe no. " << (0xF & params.m_sfnSf));
        itMap = m_allocationMaps.find(params.m_sfnSf);
        if (itMap == m_allocationMaps.end())
        {
            return;
        }
        for (uint32_t i = 0; i < (*itMap).second.size(); i++)
        {
            // convert from fixed point notation Sxxxxxxxxxxx.xxx to double
            double sinr = LteFfConverter::fpS11dot3toDouble(params.m_ulCqi.m_sinr.at(i));
            itCqi = m_ueCqi.find((*itMap).second.at(i));
            if (itCqi == m_ueCqi.end())
            {
                // create a new entry
                std::vector<double> newCqi;
                for (uint32_t j = 0; j < m_cschedCellConfig.m_ulBandwidth; j++)
                {
                    if (i == j)
                    {
                        newCqi.push_back(sinr);
                    }
                    else
                    {
                        // initialize with NO_SINR value.
                        newCqi.push_back(NO_SINR);
                    }
                }
                m_ueCqi.insert(
                    std::pair<uint16_t, std::vector<double>>((*itMap).second.at(i), newCqi));
                // generate correspondent timer
                m_ueCqiTimers.insert(
                    std::pair<uint16_t, uint32_t>((*itMap).second.at(i), m_cqiTimersThreshold));
            }
            else
            {
                // update the value
                (*itCqi).second.at(i) = sinr;
                NS_LOG_DEBUG(this << " RNTI " << (*itMap).second.at(i) << " RB " << i << " SINR "
                                  << sinr);
                // update correspondent timer
                std::map<uint16_t, uint32_t>::iterator itTimers;
                itTimers = m_ueCqiTimers.find((*itMap).second.at(i));
                (*itTimers).second = m_cqiTimersThreshold;
            }
        }
        // remove obsolete info on allocation
        m_allocationMaps.erase(itMap);
    }
    break;
    case UlCqi_s::SRS: {
        // get the RNTI from vendor specific parameters
        uint16_t rnti = 0;
        NS_ASSERT(params.m_vendorSpecificList.size() > 0);
        for (std::size_t i = 0; i < params.m_vendorSpecificList.size(); i++)
        {
            if (params.m_vendorSpecificList.at(i).m_type == SRS_CQI_RNTI_VSP)
            {
                Ptr<SrsCqiRntiVsp> vsp =
                    DynamicCast<SrsCqiRntiVsp>(params.m_vendorSpecificList.at(i).m_value);
                rnti = vsp->GetRnti();
            }
        }
        std::map<uint16_t, std::vector<double>>::iterator itCqi;
        itCqi = m_ueCqi.find(rnti);
        if (itCqi == m_ueCqi.end())
        {
            // create a new entry
            std::vector<double> newCqi;
            for (uint32_t j = 0; j < m_cschedCellConfig.m_ulBandwidth; j++)
            {
                double sinr = LteFfConverter::fpS11dot3toDouble(params.m_ulCqi.m_sinr.at(j));
                newCqi.push_back(sinr);
                NS_LOG_INFO(this << " RNTI " << rnti << " new SRS-CQI for RB  " << j << " value "
                                 << sinr);
            }
            m_ueCqi.insert(std::pair<uint16_t, std::vector<double>>(rnti, newCqi));
            // generate correspondent timer
            m_ueCqiTimers.insert(std::pair<uint16_t, uint32_t>(rnti, m_cqiTimersThreshold));
        }
        else
        {
            // update the values
            for (uint32_t j = 0; j < m_cschedCellConfig.m_ulBandwidth; j++)
            {
                double sinr = LteFfConverter::fpS11dot3toDouble(params.m_ulCqi.m_sinr.at(j));
                (*itCqi).second.at(j) = sinr;
                NS_LOG_INFO(this << " RNTI " << rnti << " update SRS-CQI for RB  " << j << " value "
                                 << sinr);
            }
            // update correspondent timer
            std::map<uint16_t, uint32_t>::iterator itTimers;
            itTimers = m_ueCqiTimers.find(rnti);
            (*itTimers).second = m_cqiTimersThreshold;
        }
    }
    break;
    case UlCqi_s::PUCCH_1:
    case UlCqi_s::PUCCH_2:
    case UlCqi_s::PRACH: {
        NS_FATAL_ERROR("TdMtFfMacScheduler supports only PUSCH and SRS UL-CQIs");
    }
    break;
    default:
        NS_FATAL_ERROR("Unknown type of UL-CQI");
    }
}

void
TdMtFfMacScheduler::RefreshDlCqiMaps()
{
    // refresh DL CQI P01 Map
    std::map<uint16_t, uint32_t>::iterator itP10 = m_p10CqiTimers.begin();
    while (itP10 != m_p10CqiTimers.end())
    {
        NS_LOG_INFO(this << " P10-CQI for user " << (*itP10).first << " is "
                         << (uint32_t)(*itP10).second << " thr " << (uint32_t)m_cqiTimersThreshold);
        if ((*itP10).second == 0)
        {
            // delete correspondent entries
            std::map<uint16_t, uint8_t>::iterator itMap = m_p10CqiRxed.find((*itP10).first);
            NS_ASSERT_MSG(itMap != m_p10CqiRxed.end(),
                          " Does not find CQI report for user " << (*itP10).first);
            NS_LOG_INFO(this << " P10-CQI expired for user " << (*itP10).first);
            m_p10CqiRxed.erase(itMap);
            std::map<uint16_t, uint32_t>::iterator temp = itP10;
            itP10++;
            m_p10CqiTimers.erase(temp);
        }
        else
        {
            (*itP10).second--;
            itP10++;
        }
    }

    // refresh DL CQI A30 Map
    std::map<uint16_t, uint32_t>::iterator itA30 = m_a30CqiTimers.begin();
    while (itA30 != m_a30CqiTimers.end())
    {
        NS_LOG_INFO(this << " A30-CQI for user " << (*itA30).first << " is "
                         << (uint32_t)(*itA30).second << " thr " << (uint32_t)m_cqiTimersThreshold);
        if ((*itA30).second == 0)
        {
            // delete correspondent entries
            std::map<uint16_t, SbMeasResult_s>::iterator itMap = m_a30CqiRxed.find((*itA30).first);
            NS_ASSERT_MSG(itMap != m_a30CqiRxed.end(),
                          " Does not find CQI report for user " << (*itA30).first);
            NS_LOG_INFO(this << " A30-CQI expired for user " << (*itA30).first);
            m_a30CqiRxed.erase(itMap);
            std::map<uint16_t, uint32_t>::iterator temp = itA30;
            itA30++;
            m_a30CqiTimers.erase(temp);
        }
        else
        {
            (*itA30).second--;
            itA30++;
        }
    }
}

void
TdMtFfMacScheduler::RefreshUlCqiMaps()
{
    // refresh UL CQI  Map
    std::map<uint16_t, uint32_t>::iterator itUl = m_ueCqiTimers.begin();
    while (itUl != m_ueCqiTimers.end())
    {
        NS_LOG_INFO(this << " UL-CQI for user " << (*itUl).first << " is "
                         << (uint32_t)(*itUl).second << " thr " << (uint32_t)m_cqiTimersThreshold);
        if ((*itUl).second == 0)
        {
            // delete correspondent entries
            std::map<uint16_t, std::vector<double>>::iterator itMap = m_ueCqi.find((*itUl).first);
            NS_ASSERT_MSG(itMap != m_ueCqi.end(),
                          " Does not find CQI report for user " << (*itUl).first);
            NS_LOG_INFO(this << " UL-CQI exired for user " << (*itUl).first);
            (*itMap).second.clear();
            m_ueCqi.erase(itMap);
            std::map<uint16_t, uint32_t>::iterator temp = itUl;
            itUl++;
            m_ueCqiTimers.erase(temp);
        }
        else
        {
            (*itUl).second--;
            itUl++;
        }
    }
}

void
TdMtFfMacScheduler::UpdateDlRlcBufferInfo(uint16_t rnti, uint8_t lcid, uint16_t size)
{
    std::map<LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator it;
    LteFlowId_t flow(rnti, lcid);
    it = m_rlcBufferReq.find(flow);
    if (it != m_rlcBufferReq.end())
    {
        NS_LOG_INFO(this << " UE " << rnti << " LC " << (uint16_t)lcid << " txqueue "
                         << (*it).second.m_rlcTransmissionQueueSize << " retxqueue "
                         << (*it).second.m_rlcRetransmissionQueueSize << " status "
                         << (*it).second.m_rlcStatusPduSize << " decrease " << size);
        // Update queues: RLC tx order Status, ReTx, Tx
        // Update status queue
        if (((*it).second.m_rlcStatusPduSize > 0) && (size >= (*it).second.m_rlcStatusPduSize))
        {
            (*it).second.m_rlcStatusPduSize = 0;
        }
        else if (((*it).second.m_rlcRetransmissionQueueSize > 0) &&
                 (size >= (*it).second.m_rlcRetransmissionQueueSize))
        {
            (*it).second.m_rlcRetransmissionQueueSize = 0;
        }
        else if ((*it).second.m_rlcTransmissionQueueSize > 0)
        {
            uint32_t rlcOverhead;
            if (lcid == 1)
            {
                // for SRB1 (using RLC AM) it's better to
                // overestimate RLC overhead rather than
                // underestimate it and risk unneeded
                // segmentation which increases delay
                rlcOverhead = 4;
            }
            else
            {
                // minimum RLC overhead due to header
                rlcOverhead = 2;
            }
            // update transmission queue
            if ((*it).second.m_rlcTransmissionQueueSize <= size - rlcOverhead)
            {
                (*it).second.m_rlcTransmissionQueueSize = 0;
            }
            else
            {
                (*it).second.m_rlcTransmissionQueueSize -= size - rlcOverhead;
            }
        }
    }
    else
    {
        NS_LOG_ERROR(this << " Does not find DL RLC Buffer Report of UE " << rnti);
    }
}

void
TdMtFfMacScheduler::UpdateUlRlcBufferInfo(uint16_t rnti, uint16_t size)
{
    size = size - 2; // remove the minimum RLC overhead
    std::map<uint16_t, uint32_t>::iterator it = m_ceBsrRxed.find(rnti);
    if (it != m_ceBsrRxed.end())
    {
        NS_LOG_INFO(this << " UE " << rnti << " size " << size << " BSR " << (*it).second);
        if ((*it).second >= size)
        {
            (*it).second -= size;
        }
        else
        {
            (*it).second = 0;
        }
    }
    else
    {
        NS_LOG_ERROR(this << " Does not find BSR report info of UE " << rnti);
    }
}

void
TdMtFfMacScheduler::TransmissionModeConfigurationUpdate(uint16_t rnti, uint8_t txMode)
{
    NS_LOG_FUNCTION(this << " RNTI " << rnti << " txMode " << (uint16_t)txMode);
    FfMacCschedSapUser::CschedUeConfigUpdateIndParameters params;
    params.m_rnti = rnti;
    params.m_transmissionMode = txMode;
    m_cschedSapUser->CschedUeConfigUpdateInd(params);
}

} // namespace ns3
