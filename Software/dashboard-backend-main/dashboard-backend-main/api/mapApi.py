from models.api.beacon import Beacon as ApiBeacon
from models.api.click import Click as ApiClick
from models.api.clickBeacon import ClickBeacon
from flask import Blueprint, request

from functions.clickFunctions import getAllClicks, getAllClickValues
from functions.beaconFunctions import getAll as getAllBeacons

from models.api.clickLocations import ClickWithLocation

import statistics

from functions.json import json


MAX_BEACON_COUNT = 50
DISTANCE_CONSTANT = 1
X_CONSTANT = 5
Y_CONSTANT = 3


app = Blueprint("map", __name__)

@app.route("")
def getClicksOnMap():
    clicks: list[ApiClick]  = getAllClickValues() 

    clickLocations = displayMap(clicks)

    return json(clickLocations), 200


def generateIndecesArray(beaconAmount: int):
    beaconAmount = min(beaconAmount, MAX_BEACON_COUNT)
    if beaconAmount < 3:
        return []
    arr = []
    for i in range(beaconAmount):
        if i < 2:
            continue
        for j in range(i):
            if j < 1:
                continue
            for k in range(j):
                arr.append([k,j,i])
    print(arr)
    return arr

if __name__ == '__main__':
    print(generateIndecesArray(6))
    print(len(generateIndecesArray(6)))

def displayMap(clicks: list[ApiClick]):
    beaconsWithLocations = []

    for click in clicks:
        clickBeacons = click.click_beacons
        click.click_time = click.click_time.isoformat()

        beaconLocationsX = []
        beaconLocationsY = []

        if len(clickBeacons) < 3:
            continue

        for set in generateIndecesArray(len(clickBeacons)):
            beacon1 = clickBeacons[set[0]]
            print(beacon1['distance'])
            beacon2 = clickBeacons[set[1]]
            beacon3 = clickBeacons[set[2]]

            distance1 = beacon1['distance'] * DISTANCE_CONSTANT
            distance2 = beacon2['distance'] * DISTANCE_CONSTANT
            distance3 = beacon3['distance'] * DISTANCE_CONSTANT
            x1 = beacon1['beacon'].x * X_CONSTANT
            x2 = beacon2['beacon'].x * X_CONSTANT
            x3 = beacon3['beacon'].x * X_CONSTANT

            y1 = beacon1['beacon'].y * Y_CONSTANT
            y2 = beacon2['beacon'].y * Y_CONSTANT
            y3 = beacon3['beacon'].y * Y_CONSTANT

            A = -2 * x1 + 2 * x2
            B = -2 * y1 + 2 * y2
            C = distance1 ** 2 - distance2 ** 2 - x1 ** 2 + x2 ** 2 - y1 ** 2 + y2 ** 2

            D = -2 * x2 + 2 * x3
            E = -2 * y2 + 2 * y3
            F = distance2 ** 2 - distance3 ** 2 - x2 ** 2 + x3 ** 2 - y2 ** 2 + y3 ** 2
            
            clickX = (C * E - F * B) / (E * A - D * B)
            clickY = (C * D - A * F) / (B * D - A * E)

            beaconLocationsX.append(clickX)
            beaconLocationsY.append(clickY)

        beaconLocationsX.sort()
        beaconLocationsY.sort()
        
        beaconWithLocation = ClickWithLocation()
        beaconWithLocation.click = click
        beaconWithLocation.x = statistics.median(beaconLocationsX)
        beaconWithLocation.y = statistics.median(beaconLocationsY)

        beaconsWithLocations.append(beaconWithLocation)

    return beaconsWithLocations





def returnOrder(beacon: ClickBeacon):
    return beacon.distance
