#!/usr/bin/python

"""
Setting mechanism to optimize the use of the APs
"""

import time

from mininet.node import Controller
from mininet.log import setLogLevel, debug
from mn_wifi.cli import CLI_wifi
from mn_wifi.net import Mininet_wifi
from mn_wifi.node import OVSKernelAP
from util import _parseIperf, sleeptime, setSNR, loadness, rssi_tag

from RL_brain import DeepQNetwork
from mn_wifi.link import Association


def iperf(hosts=None, l4Type='TCP', udpBw='10M',
          seconds=5, port=5001):

    hosts = hosts or [hosts[0], hosts[-1]]
    assert len(hosts) == 2
    client, server = hosts

    server.cmd('killall -9 iperf')
    iperfArgs = 'iperf -p %d ' % port
    bwArgs = ''
    if l4Type == 'UDP':
        iperfArgs += '-u '
        bwArgs = '-b ' + udpBw + ' '

    server.sendCmd(iperfArgs + '-s')
    cliout = client.cmd(iperfArgs + '-t %d -c ' % seconds +
                        server.IP() + ' ' + bwArgs)
    debug('Client output: %s\n' % cliout)
    servout = ''
    count = 2 if l4Type == 'TCP' else 1

    server.sendInt()
    servout += server.waitOutput()
    debug('Server output: %s\n' % servout)
    result = [_parseIperf(servout), _parseIperf(cliout)]
    return result[0]


def _getreward(station, host):
    reward_dic = []
    reward_dic.append(iperf([station, host], l4Type='TCP', seconds=0.00001, port=6001))
    # print reward_dic[-1]

    return reward_dic[-1]
    # if len(reward_dic) == 3:
        # print reward_dic[-1]
        # return reward_dic[-1]
        # sum(reward_dic)/len(reward_dict)


def handover(sta, ap, wlan):
    changeAP = True
    """Association Control: mechanisms that optimize the use of the APs"""
    if sta.params['associatedTo'][wlan] == '' or changeAP == True:
        if ap not in sta.params['associatedTo']:
            cls = Association
            # debug('iwconfig %s essid %s ap %s\n' % (sta.params['wlan'][wlan], ap.params['ssid'][0], \
            #                                           ap.params['mac'][0]))
            # sta.pexec('iwconfig %s essid %s ap %s' % (sta.params['wlan'][wlan], ap.params['ssid'][0], \
            #                                           ap.params['mac'][0])
            # cls.associate_noEncrypt(sta, ap, wlan)
            # mobility.updateAssociation(sta, ap, wlan)
            cls.update(sta, ap, wlan)


def step(currentID, action, sta14, ap1, ap2, ap3, h1):
    actionID = action.argmax()
    n_APs = 3

    if actionID != currentID and actionID <= n_APs:
        if actionID==0:
            handover(sta14, ap1, 0)
        elif actionID==1:
            handover(sta14, ap2, 0)
        elif actionID==2:
            handover(sta14, ap3, 0)
    reward = _getreward(sta14, h1)
    nextstate = [chanFunt(ap1, sta14), chanFunt(ap2, sta14), chanFunt(ap3, sta14)]

    return reward, nextstate


def chanFunt(new_ap, new_st):
    """collect rssi from aps to station
       :param new_ap: access point
       :param new_st: station
    """
    APS = ['ap1', 'ap2', 'ap3', 'ap4', 'ap5', 'ap6', 'ap7', 'ap8', 'ap9']
    for number in APS:
        if number == str(new_ap):
            indent = 0
            for item in new_st.params['apsInRange']:
                if number in str(item):
                    for each_item in new_ap.params['stationsInRange']:
                        if str(new_st) in str(each_item):
                            return new_ap.params['stationsInRange'][each_item]
                        else:
                            pass
                else:
                    indent = indent + 1
            if indent == len(new_st.params['apsInRange']):
                return 0
        else:
            pass

def topology():
    "Create a network."
    net = Mininet_wifi(controller=Controller, accessPoint=OVSKernelAP)

    print "*** Creating nodes"
    h1 = net.addHost('h1', mac='00:00:00:00:00:01', ip='10.0.0.1/8')
    sta14 = net.addStation('sta1', mac='00:00:00:00:00:02', ip='10.0.0.2/8', position='70,50,0')

    ap1 = net.addAccessPoint('ap1', ssid='ssid-ap1', mode='g', channel='1',
                             position='50,50,0')
    ap2 = net.addAccessPoint('ap2', ssid='ssid-ap2', mode='g', channel='6',
                             position='70,50,0', range=30)
    ap3 = net.addAccessPoint('ap3', ssid='ssid-ap3', mode='g', channel='11',
                             position='90,50,0')
    c1 = net.addController('c1', controller=Controller)

    net.setPropagationModel(model="logDistance", exp = 5)
    print "*** Configuring wifi nodes"
    net.configureWifiNodes()

    print "*** Associating and Creating links"
    net.addLink(h1, ap1)
    net.addLink(ap1, ap2)
    net.addLink(ap2, ap3)
    # net.addLink(ap1, sta14)

    """uncomment to plot graph"""
    # net.plotGraph(max_x=400, max_y=400)
    net.plotGraph(max_x=120, max_y=120)

    net.setMobilityModel(time=0, model='RandomWayPoint', max_x=120, max_y=120,
                         min_v=30, max_v=50, seed=5, ac_method='ssf')

    print "*** Starting network"
    net.build()
    c1.start()
    ap1.start([c1])
    ap2.start([c1])
    ap3.start([c1])

    print "*** Running CLI"
    # CLI_wifi(net)
    second = sleeptime(0, 0, 1)

    new_rssi = [chanFunt(ap1,sta14),chanFunt(ap2,sta14),chanFunt(ap3,sta14)]
    print new_rssi

    n_actions, n_APs = len(new_rssi), len(new_rssi)
    brain = DeepQNetwork(n_actions,n_APs,param_file = None)

    state = new_rssi
    print 'initial observation:' + str(state)


    try:
        while True:
            time.sleep(second)
            new_rssi = [chanFunt(ap1, sta14), chanFunt(ap2, sta14), chanFunt(ap3, sta14)]
    #         # print new_rssi, rssi_tag(sta14)
    #         # print _getreward(sta14,h1)
    #         # print iperf([sta14, h1], seconds=0.0000001)
    #         # print '*********############*'
            action,q_value = brain.choose_action(state)
            reward, nextstate = step(rssi_tag(sta14),action, sta14, ap1, ap2, ap3, h1)
    #
    #         print 'iperf' + iperf([sta14, h1])
    #         print new_rssi
            brain.setPerception(state, action, reward, nextstate)
            state = nextstate
    except KeyboardInterrupt:
        print 'saving replayMemory...'
        brain.saveReplayMemory()
    pass
    # # print new_rssi
    # # snr_dict = map(setSNR,new_rssi)
    #
    # print "*** Stopping network"
    net.stop()

if __name__ == '__main__':
    setLogLevel('info')
    topology()
