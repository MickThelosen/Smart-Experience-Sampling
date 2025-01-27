import { Beacon, MapBeacon } from "@/types/beacon"
import { API_URL } from "../constants"

export default async function getBeaconsFromLocation(locationId: string) {
    let headers = new Headers()
    headers.append('Access-Control-Allow-Origin', 'http://localhost:3000')
    const response = await fetch(`${API_URL}/api/beacon/location/${locationId}`, {
        headers: headers
    })
    const dataJson: string[] = await response.json()

    let data: MapBeacon[] = []
    dataJson.forEach(element => {
        let research = (JSON.parse(element) as MapBeacon)
        data.push(research)
    })
    return data
}