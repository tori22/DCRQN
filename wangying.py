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
from mn_wifi.link import wmediumd
from mn_wifi.wmediumdConnector import interference

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
    # print servout
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

def get_state(ap):
    return ap.params['channel'][0], ap.params['txpower'][0]

def topology():
    "Create a network."
    net = Mininet_wifi(controller=Controller, accessPoint=OVSKernelAP,
                       link=wmediumd, wmediumd_mode=interference, noise_threshold=-91, fading_coefficient=3)

    print "*** Creating nodes"
    h1 = net.addHost('h1', mac='00:00:00:00:00:01', ip='10.0.0.1/8')
    sta1 = net.addStation('sta1', mac='00:00:00:00:00:02', ip='10.0.0.2/8', position='70,50,0')
    sta2 = net.addStation('sta2', mac='00:00:00:00:00:03', ip='10.0.0.3/8', position='80,60,0')
    sta3 = net.addStation('sta3', mac='00:00:00:00:00:04', ip='10.0.0.4/8', position='90,70,0')
    sta4 = net.addStation('sta4', mac='00:00:00:00:00:05', ip='10.0.0.5/8', position='100,80,0')
    sta5 = net.addStation('sta5', mac='00:00:00:00:00:06', ip='10.0.0.6/8', position='110,90,0')
    sta6 = net.addStation('sta6', mac='00:00:00:00:00:07', ip='10.0.0.7/8', position='120,20,0')
    sta7 = net.addStation('sta7', mac='00:00:00:00:00:08', ip='10.0.0.8/8', position='130,30,0')
    sta8 = net.addStation('sta8', mac='00:00:00:00:00:09', ip='10.0.0.9/8', position='140,20,0')


    ap1 = net.addAccessPoint('ap1', ssid='ssid-ap1', mode='g', channel='1',
                             position='50,50,0')
    ap2 = net.addAccessPoint('ap2', ssid='ssid-ap2', mode='g', channel='1',
                             position='70,60,0', range=30)
    ap3 = net.addAccessPoint('ap3', ssid='ssid-ap3', mode='g', channel='1',
                             position='90,90,0')
    ap4 = net.addAccessPoint('ap4', ssid='ssid-ap4', mode='g', channel='1',
                             position='110,70,0')
    ap5 = net.addAccessPoint('ap5', ssid='ssid-ap5', mode='g', channel='1',
                             position='130,10,0')
    ap6 = net.addAccessPoint('ap6', ssid='ssid-ap6', mode='g', channel='1',
                             position='150,20,0')
    ap7 = net.addAccessPoint('ap7', ssid='ssid-ap7', mode='g', channel='1',
                             position='170,30,0')
    ap8 = net.addAccessPoint('ap8', ssid='ssid-ap8', mode='g', channel='1',
                             position='190,40,0')
    c1 = net.addController('c1', controller=Controller)

    net.setPropagationModel(model="logDistance", exp = 5)
    print "*** Configuring wifi nodes"
    net.configureWifiNodes()

    print "*** Associating and Creating links"
    net.addLink(h1, ap1)
    net.addLink(ap1, ap2)
    net.addLink(ap2, ap3)
    net.addLink(ap3, ap4)
    net.addLink(ap4, ap5)
    net.addLink(ap5, ap6)
    net.addLink(ap6, ap7)
    net.addLink(ap7, ap8)

    """uncomment to plot graph"""
    # net.plotGraph(max_x=400, max_y=400)
    net.plotGraph(max_x=250, max_y=150)

    net.setMobilityModel(time=0, model='RandomWayPoint', max_x=250, max_y=150,
                         min_v=3, max_v=5, seed=5, ac_method='ssf')

    print "*** Starting network"
    net.build()
    c1.start()
    ap1.start([c1])
    ap2.start([c1])
    ap3.start([c1])
    ap4.start([c1])
    ap5.start([c1])
    ap6.start([c1])
    ap7.start([c1])
    ap8.start([c1])

    print "*** Running CLI"
    print iperf([sta1, h1])
    print iperf([sta2, h1])
    print iperf([sta3, h1])
    print iperf([sta4, h1])
    print iperf([sta5, h1])
    print iperf([sta6, h1])
    print iperf([sta7, h1])
    print iperf([sta8, h1])

    CLI_wifi(net)

    state = [get_state(ap) for ap in [ap1, ap2, ap3, ap4, ap5, ap6, ap7, ap8]]
    print state
    n_actions = len(state) * 2
    n_APs = len(state)
    brain = DeepQNetwork(n_actions, n_APs, param_file= None)
    # second = sleeptime(0, 0, 1)
    #
    # new_rssi = [chanFunt(ap1,sta14),chanFunt(ap2,sta14),chanFunt(ap3,sta14)]
    # print new_rssi
    #
    # n_actions, n_APs = len(new_rssi), len(new_rssi)
    # brain = DeepQNetwork(n_actions,n_APs,param_file = None)
    #
    # state = new_rssi
    # print 'initial observation:' + str(state)
    #
    #
    # try:
    #     while True:
    #         time.sleep(second)
    #         new_rssi = [chanFunt(ap1, sta14), chanFunt(ap2, sta14), chanFunt(ap3, sta14)]
    # #         # print new_rssi, rssi_tag(sta14)
    # #         # print _getreward(sta14,h1)
    # #         # print iperf([sta14, h1], seconds=0.0000001)
    # #         # print '*********############*'
    #         action,q_value = brain.choose_action(state)
    #         reward, nextstate = step(rssi_tag(sta14),action, sta14, ap1, ap2, ap3, h1)
    # #
    # #         print 'iperf' + iperf([sta14, h1])
    # #         print new_rssi
    #         brain.setPerception(state, action, reward, nextstate)
    #         state = nextstate
    # except KeyboardInterrupt:
    #     print 'saving replayMemory...'
    #     brain.saveReplayMemory()
    # pass
    # # print new_rssi
    # # snr_dict = map(setSNR,new_rssi)
    #
    # print "*** Stopping network"
    net.stop()

if __name__ == '__main__':
    setLogLevel('info')
    topology()
