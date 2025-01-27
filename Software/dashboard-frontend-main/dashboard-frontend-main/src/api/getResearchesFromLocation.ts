
import { Research } from "@/types/research"
import { API_URL } from "../constants"

export default async function getResearchesFromLocation(id: string) {
    let headers = new Headers()
    headers.append('Access-Control-Allow-Origin', 'http://localhost:3000')
    const response = await fetch(`${API_URL}/api/research/${id}`, {
        headers: headers
    })
    const dataJson: string[] = await response.json()

    let data: Research[] = []
    dataJson.forEach(element => {
        let research = (JSON.parse(element) as Research)
        research.startTime = new Date(research.startTime)
        research.endTime = new Date(research.endTime)
        data.push(research)
    })
    return data
}