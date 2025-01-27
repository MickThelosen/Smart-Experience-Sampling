import { API_URL } from "@/constants"

export default async function saveLocation(name: string, parentId: string | null, image?: File) {
    let headers = new Headers()
    let formData = new FormData()
    formData.append("name", name)
    formData.append("parentId", parentId || "")
    formData.append("file", image || "")
        headers.append('Access-Control-Allow-Origin', 'http://localhost:3000')
        headers.append('Content-Type', 'application/x-www-form-urlencoded')
        const response = await fetch(`${API_URL}/api/location`, {
            headers: headers,
            method: 'POST',
            body: formData
        })
    }