import { API_URL } from "@/constants"

export default async function createResearch(question: string, startTime: string, endTime: string, id: string) {
    let headers = new Headers()
    let inputData: any = {
        question: question,
        startTime: startTime,
        endTime: endTime,
        locationId: id
    }
    let input = JSON.stringify(inputData)
        headers.append('Access-Control-Allow-Origin', 'http://localhost:3000')
        headers.append('Content-Type', 'application/json')
        const response = await fetch(`${API_URL}/api/research`, {
            headers: headers,
            method: 'POST',
            body: input
        })
    }