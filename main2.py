#!/usr/bin/python

"""
Setting mechanism to optimize the use of the APs
"""

import time
import math
from mininet.node import Controller
from mininet.log import setLogLevel, debug
from mn_wifi.cli import CLI_wifi
from mn_wifi.net import Mininet_wifi
from mn_wifi.node import OVSKernelAP
from util import _parseIperf, sleeptime, setSNR, loadness, rssi_tag, neighborAp
from mn_wifi.link import wmediumd
from mn_wifi.wmediumdConnector import interference

from RL_brain import DeepQNetwork
from mn_wifi.link import Association
import numpy as np

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

def step(state, action, ap1, ap2, ap3, ap4, ap5, ap6, ap7, ap8, sta1, sta2, sta3, sta4, sta5, sta6, sta7, sta8, h1):
    actionID = np.argmax(action)
    print 'choose action: ' + str(actionID)
    actionID = int(actionID)
    apIndex = actionID / 4
    channel_power_index = actionID % 4

    rewardorigin = caculate(sta1, sta2, sta3, sta4, sta5, sta6, sta7, sta8, ap1, ap2, ap3, ap4, ap5, ap6, ap7, ap8)
    apArray = [ap1, ap2, ap3, ap4, ap5, ap6, ap7, ap8]
    ap = apArray[apIndex]
    if channel_power_index == 0:
        originChannel = int(ap.params['channel'][0])
        if originChannel == 14:
            ap.setChannel(1, intf=ap.params['wlan'][0])
        else:
            ap.setChannel(str(originChannel+1), intf=ap.params['wlan'][0])
        state[apIndex*2] = ap.params['channel'][0]
    elif channel_power_index == 1:
        originChannel = int(ap.params['channel'][0])
        if originChannel == 1:
            ap.setChannel(14, intf=ap.params['wlan'][0])
        else:
            ap.setChannel(str(originChannel - 1), intf=ap.params['wlan'][0])
        state[apIndex * 2] = ap.params['channel'][0]
    elif channel_power_index == 2:
        originPower = int(ap.params['txpower'][0])
        if originPower == 14:
            ap.setTxPower(1, intf=ap.params['wlan'][0])
        else:
            ap.setTxPower(ap.params['txpower'][0] + 1, intf=ap.params['wlan'][0])
        # ap.setRange(ap.params['range']+1, intf=ap.params['wlan'][0])
        state[apIndex * 2 + 1] = ap.params['txpower'][0]
    else:
        originPower = int(ap.params['txpower'][0])
        if originPower==1:
            ap.setTxPower(14, intf=ap.params['wlan'][0])
        else:
            ap.setTxPower(ap.params['txpower'][0] - 1, intf=ap.params['wlan'][0])
        # ap.setRange(ap.params['range'] - 1, intf=ap.params['wlan'][0])
        state[apIndex * 2 + 1] = ap.params['txpower'][0]

    # reward = [float(iperf([sta1, h1])), float(iperf([sta2, h1])),
    #           float(iperf([sta3, h1])), float(iperf([sta4, h1])),
    #           float(iperf([sta5, h1])), float(iperf([sta6, h1])),
    #           float(iperf([sta7, h1])), float(iperf([sta8, h1]))]
    rewardnow = caculate(sta1, sta2, sta3, sta4, sta5, sta6, sta7, sta8, ap1, ap2, ap3, ap4, ap5, ap6, ap7, ap8)
    reward = rewardnow - rewardorigin
    print reward
    return reward, state, rewardnow

def caculate(sta1, sta2, sta3, sta4, sta5, sta6, sta7, sta8, ap1, ap2, ap3, ap4, ap5, ap6, ap7, ap8):
    totalReward = 0
    stationSet = [sta1, sta2, sta3, sta4, sta5, sta6, sta7, sta8]
    apSet = [ap1, ap2, ap3, ap4, ap5, ap6, ap7, ap8]
    for station in stationSet:
        apIndex = rssi_tag(station)
        stationPosition = station.params['position']

        apPositive = apSet[apIndex]
        apPower = apPositive.params['txpower'][0]
        apPosition = apPositive.params['position']
        distance = getDistance(apPosition, stationPosition)
        channel = apPositive.params['channel'][0]
        interference = 0
        neighbors = neighborAp(station)
        for apIndexNegative in neighbors:
            if apIndexNegative == apIndex:
                continue
            apNegative = apSet[apIndexNegative]
            channelNegative = apNegative.params['channel'][0]
            if channelNegative != channel:
                continue
            apNegativePower = apNegative.params['txpower'][0]
            apNegativePosition = apNegative.params['position']
            distanceNegative = getDistance(apNegativePosition, stationPosition)
            interference = apNegativePower/distanceNegative
        totalReward += apPower/distance / (1 + interference)
    return totalReward

def getDistance(apPosition, stationPosition):
    x = apPosition[0]-stationPosition[0]
    y = apPosition[1] - stationPosition[1]
    z = apPosition[2] - stationPosition[2]
    return math.sqrt(x*x + y*y + z*z)

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
    # CLI_wifi(net)

    state = []
    for ap in [ap1, ap2, ap3, ap4, ap5, ap6, ap7, ap8]:
        temp = list(get_state(ap))
        state.append(str(temp[0]))
        state.append(str(temp[1]))
    n_actions = len(state) * 2
    n_APs = len(state)
    originState = state
    brain = DeepQNetwork(n_actions, n_APs, param_file= None)
    action, q_value = brain.choose_action(state)
    reward, nextstate, rewardnow = step(state, action, ap1, ap2, ap3, ap4, ap5, ap6, ap7, ap8, sta1, sta2, sta3, sta4, sta5, sta6, sta7, sta8, h1)

    second = sleeptime(0, 0, 1)

    rewardtrain = []
    try:
        while True:
            time.sleep(second)
            action, q_value = brain.choose_action(state)
            reward, nextstate, rewardnow = step(state, action, ap1, ap2, ap3, ap4, ap5, ap6, ap7, ap8, sta1, sta2, sta3, sta4, sta5, sta6, sta7, sta8, h1)
            brain.setPerception(state, action, reward, nextstate)
            rewardtrain.append(rewardnow)
            state = nextstate
    except KeyboardInterrupt:
        print 'saving replayMemory...'
        brain.saveReplayMemory()
        brain.plot_cost()
        np.savetxt("./rewardtrain.txt", rewardtrain, fmt='%f', delimiter=',')
    pass

    rewardresults=[]
    try:
        while True:
            action, q_value = brain.choose_action(originState)
            reward, nextstate, rewardnow = step(state, action, ap1, ap2, ap3, ap4, ap5, ap6, ap7, ap8, sta1, sta2, sta3, sta4, sta5,
                                     sta6, sta7, sta8, h1)
            originState = nextstate
            print 'reward: ' + str(rewardnow)
            print originState
            rewardresults.append(rewardnow)
    except KeyboardInterrupt:
        print 'print reward...'
        np.savetxt("./reward.txt", rewardresults, fmt='%f', delimiter=',')
    print "*** Stopping network"
    net.stop()

if __name__ == '__main__':
    setLogLevel('info')
    topology()
