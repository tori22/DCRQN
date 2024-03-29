import re
def _parseIperf(iperfOutput):
    r = r'([\d\.]+\w)'
    m = re.findall(r, iperfOutput)
    if m:
        return m[-1]
    else:
        # was: raise Exception(...)
        print ('could not parse iperf output: ' + iperfOutput)
        return ''


def sleeptime(hour, minu, sec):
    """time to output"""
    return hour * 3600 + minu * 60 + sec


def setSNR(signal):
    """
    set SNR
    :param signal: RSSI
    """
    if signal != 0:
        snr = float('%.2f' % (signal - (-91.0)))
    else:
        snr = 0
    return snr


def loadness(new_ap):

    """ return the loadness of the APs"""
    return len(new_ap.params['associatedStations'])

def rssi_tag(new_string):
    """output the number of associated AP"""
    for each_item in new_string.params['associatedTo']:
        if str('ap1') in str(each_item):
            return 0
        elif str('ap2') in str(each_item):
            return 1
        elif str('ap3') in str(each_item):
            return 2
        elif str('ap4') in str(each_item):
            return 3
        elif str('ap5') in str(each_item):
            return 4
        elif str('ap6') in str(each_item):
            return 5
        elif str('ap7') in str(each_item):
            return 6
        elif str('ap8') in str(each_item):
            return 7
        elif str('ap9') in str(each_item):
            return 8
        else:
            return 0

def neighborAp(station):
    aps = []
    neighbors = station.params['apsInRange']
    for each_item in neighbors:
        if str('ap1') in str(each_item):
            aps.append(0)
            continue
        if str('ap2') in str(each_item):
            aps.append(1)
            continue
        if str('ap3') in str(each_item):
            aps.append(2)
            continue
        if str('ap4') in str(each_item):
            aps.append(3)
            continue
        if str('ap5') in str(each_item):
            aps.append(4)
            continue
        if str('ap6') in str(each_item):
            aps.append(5)
            continue
        if str('ap7') in str(each_item):
            aps.append(6)
            continue
        if str('ap8') in str(each_item):
            aps.append(7)
            continue
        if str('ap9') in str(each_item):
            aps.append(8)
            continue
    return aps