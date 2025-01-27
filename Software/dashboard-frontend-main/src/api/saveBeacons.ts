import { API_URL } from "@/constants"
import { Beacon, MapBeacon } from "@/types/beacon"

export default async function saveAllBeacons(locationId: string, beacons: MapBeacon[]) {
    let returnedBeacons: MapBeacon[] = []
    beacons.forEach(async (beacon) => {
        let headers = new Headers()
        let inputData: any = {
            x: beacon.x,
            y: beacon.y,
            location_id: locationId
        }
        let input = JSON.stringify(inputData)
            headers.append('Access-Control-Allow-Origin', 'http://localhost:3000')
            headers.append('Content-Type', 'application/json')
            const response = await fetch(`${API_URL}/api/beacon/register/${beacon.uid}`, {
                headers: headers,
                method: 'POST',
                body: input
            })

            const data = await response.json() as MapBeacon
            returnedBeacons.push(data)
            console.log(data)
        })

        return returnedBeacons
    };
    