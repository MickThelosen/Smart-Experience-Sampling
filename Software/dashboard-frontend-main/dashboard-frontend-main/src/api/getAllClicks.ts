import { Click } from "@/types/click"
import { API_URL } from "../constants"

/**
 * gets all clicks from the database
 * @returns all clicks known in the system
 */
export async function getAllClicks() {
    let headers = new Headers()
    headers.append('Access-Control-Allow-Origin', 'http://localhost:3000')
    const response = await fetch(`${API_URL}/api/map`, {
        method: "GET",
        headers: headers
    })
    const data: Click[] = await response.json()
    console.log(data)
    return data
}