import {useState, useEffect} from 'react'
import { io, Socket } from 'socket.io-client'

/**
 * On mount, connect to the socket.io server
 * @param {*} url
 * @param {*} handleData
 * @returns
 */

export default function useWebSocket(url: string | undefined, handleData: Function) {
    const [socket, setSocket] = useState<any>(null)
    useEffect(() => {
        const socketClient = io(url)

        socketClient.on("connect", () => {
            console.log('Connected to server')
        })

        socketClient.on("disconnect", () => {
            console.log('Disconnected from server')
        })

        socketClient.on('data', (data) => {
            handleData(data)
        })

        setSocket(socketClient)

        return () => {
            socketClient.close()
        }
    }, [])

    return socket
}