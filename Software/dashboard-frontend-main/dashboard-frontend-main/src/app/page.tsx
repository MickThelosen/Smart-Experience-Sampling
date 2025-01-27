'use client'

import { Click } from "@/types/click";
import ClickTable from "../components/click-table";
import { useEffect, useState } from "react";
import { getAllClicks } from "@/api/getAllClicks";
import useWebSocket from "../hooks/useWebSocket";
import { API_URL } from "@/constants";
import Map from "@/components/map/map";

export default function Home() {
  const [clicks, setClicks] = useState<Click[]>([])

  const socket = useWebSocket(
    API_URL, (data: any) => {
      let click: Click = JSON.parse(data)
      console.log('new click', click)
      setClicks(prev => [...prev, click])
    }
  )

  useEffect(() => {
    async function getClicks() {
      
      let clicks = await getAllClicks()
      setClicks(clicks)
    }

    getClicks()
  }, [])
  return (
    <Map />
    // <div style={{display: 'flex'}}>
    // <div className="navbar" style={{backgroundColor: 'red', width: '200px'}}>
    //   <div></div>
    // </div>
    // <main className="container mx-auto" style={{padding: '40px'}}>
    //   <h1 className="text-2xl font-bold mb-4">Kan je geconcentreerd werken op jouw werkplek?</h1>
    //   <ClickTable clicks={clicks} />
    // </main>
    // </div>
  );
}
