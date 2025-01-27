import { Location } from "@/types/location"
import { API_URL } from "../../constants"

/**
 * gets all clicks from the database
 * @returns all clicks known in the system
 */
export async function getAllLocations() {
    let headers = new Headers()
    headers.append('Access-Control-Allow-Origin', 'http://localhost:3000')
    const response = await fetch(`${API_URL}/api/location`, {
        headers: headers
    })
    const data: Location[] = await response.json()
    return data
}