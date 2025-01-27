'use client'

import { ChangeEvent, useEffect, useState } from 'react'
import './map.css'
import { MapBeacon } from '@/types/beacon'

import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { Button } from '../ui/button'
import { MapPin } from 'lucide-react'
import NewMapDialog from './NewMapDialog'
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { flattenLocations } from '@/app/locations/createLocationModal'
import { getRootLocations } from '@/api/location/getRootLocations'
import { Location } from '@/types/location'
import { Research } from '@/types/research'
import getResearchesFromLocation from '@/api/getResearchesFromLocation'
import saveAllBeacons from '@/api/saveBeacons'
import getBeaconsFromLocation from '@/api/getBeaconsFromLocation'
import { getAllClicks } from '@/api/getAllClicks'
import useWebSocket from '@/hooks/useWebSocket'
import { API_URL } from '@/constants'
import { Click } from '@/types/click'



export default function Map() {
    const [mapImage, setMapImage] = useState<string | null>(null)
    const [beacons, setBeacons] = useState<MapBeacon[]>([])
    const [newBeacons, setNewBeacons] = useState<MapBeacon[]>([])
    const [addingBeacons, setAddingBeacons] = useState<boolean>(false)
    const [creatingBeacon, setCreatingBeacon] = useState<boolean>(false)
    const [tempBeacon, setTempBeacon] = useState<any>(null)
    const [researchId, setResearchId] = useState<string | null>(null)
    const [locationId, setLocationId] = useState<string | null>(null)
    const [clicks, setClicks] = useState<Click[]>([])
    const [researches, setResearches] = useState<Research[]>([])
    const [locations, setLocations] = useState<Location[]>([])
    const flattenedLocations = flattenLocations(locations)
    useEffect(() => {
        async function getLocations() {
            
            let locations = await getRootLocations()
            setClicks(await getAllClicks())
            setLocations(locations)
        }
    
        getLocations()
    }, [])

    const socket = useWebSocket(
        API_URL, (data: any) => {
          let click: Click = JSON.parse(data)
          console.log('new click', click)
          setClicks(prev => [...prev, click])
        }
      )

    async function getResearches(id: string) {
        if (id === "none") {
            setResearches([])
            return
        }  
        let researches = await getResearchesFromLocation(id)
        setResearches(researches)
        console.log(researches)
        }

    async function getBeacons(locationId: string) {
        if (locationId === "none") {
            setBeacons([])
            return
        }  
        let beacons = await getBeaconsFromLocation(locationId)
        setBeacons(beacons)
        console.log(beacons)
        }
    
    async function getClicks() {
        setClicks(await getAllClicks())
    }

    const handleFileInput = (e: ChangeEvent<HTMLInputElement>) => {
        const file = e.target.files?.[0]
        if (file) {
          const reader = new FileReader()
          reader.onload = (e) => {
            setMapImage(e.target?.result as string)
          }
          reader.readAsDataURL(file)
        }
      }

    const cancelAddBeacons = () => {
        setNewBeacons([])
        setAddingBeacons(false)
    }

    const handleMapClick = (e: React.MouseEvent<HTMLImageElement>) => {
        if (!addingBeacons) return
        const rect = e.currentTarget.getBoundingClientRect()
        const x = ((e.clientX - rect.left) / rect.width) * 100
        const y = ((e.clientY - rect.top) / rect.height) * 100
        setCreatingBeacon(true)
        setTempBeacon({x: x, y: y})
      }

    const handleCreateBeacon = (name: string) => {
        setNewBeacons((prevBeacons) => [...prevBeacons, {x: tempBeacon.x, y: tempBeacon.y, uid: name, id: null}])
    }

    const saveBeacons = async () => {
        console.log(newBeacons)
        if (locationId != null) {

            let tempBeacons: MapBeacon[] = await saveAllBeacons(locationId, newBeacons)
            console.log(tempBeacons)
            setBeacons((prevBeacons) => [...prevBeacons, ...tempBeacons])
            setNewBeacons([])
        }
    }
    
    return (
        <div className='map-wrapper'>
            <NewMapDialog isOpen={creatingBeacon} onClose={() => setCreatingBeacon(false)} onCreate={handleCreateBeacon} />
            <div className='map-header'>
                <h1>
                    {researches.filter(r => r.id == researchId)[0]?.question ?? "Question"}
                </h1>
                <div className='map-settings'>
                <p>Research:</p>
                <Select onValueChange={(value) => setResearchId(value === "none" ? null : value)} value={researchId || "none"}>
                <SelectTrigger className="col-span-3 select_location" disabled={researches.length == 0}>
                    <SelectValue placeholder="Select a parent location (optional)" />
                </SelectTrigger>
                <SelectContent>
                    {researches?.map((research) => (
                        <SelectItem key={research.id} value={research.id} className="flex items-center">
                        <div className="flex items-center">
                        {research.question}
                        </div>
                    </SelectItem>
                    ))}
                </SelectContent>
            </Select>
                <p>Location:</p>
                <Select onValueChange={(value) => {
                    setLocationId(value === "none" ? null : value)
                    getResearches(value)
                    getBeacons(value)
                    }} value={locationId || "none"}>
                <SelectTrigger className="col-span-3 select_location">
                    <SelectValue placeholder="Select a parent location (optional)" />
                </SelectTrigger>
                <SelectContent>
                    {flattenedLocations?.map((location) => (
                        <SelectItem key={location.id} value={location.id} className="flex items-center">
                        <div style={{ marginLeft: `${location.depth * 16}px` }} className="flex items-center">
                        {location.name}
                        </div>
                    </SelectItem>
                    ))}
                </SelectContent>
            </Select>
            </div>
            </div>
            <div className='map'>
                <div>

                <img src={mapImage || undefined} onClick={handleMapClick}/>
                {beacons.map((beacon, index) => {
                    console.log(beacon)
                    return <MapPin key={index} className='absolute text-lime-500' style={{
                        left: `${beacon.x}%`,
                        top: `${beacon.y}%`,
                        transform: 'translate(-50%, -100%)'
                    }} />
                })}
                {newBeacons.map((beacon, index) => (
                    
                    <MapPin key={index} className='absolute text-red-500' style={{
                        left: `${beacon.x}%`,
                        top: `${beacon.y}%`,
                        transform: 'translate(-50%, -100%)'
                    }} />
                ))}
                {clicks.map((click: any) => (
                    
                    <MapPin key={click.id} className='absolute text-red-500' style={{
                        left: `${click.x}%`,
                        top: `${click.y}%`,
                        transform: 'translate(-50%, -100%)'
                    }} />
                ))}
                </div>
            </div>
            <div className='settings'>
                <div className="grid w-full max-w-sm items-center gap-1.5 cursor-pointer text-center">
                    <Label htmlFor="picture">Picture</Label>
                    <Input id="picture" type="file" className='cursor-pointer' onChange={handleFileInput}/>
                </div>
                <div className='grid w-full max-w-sm items-center gap-1.5 text-center'>

                    <Label htmlFor='add-beacons'>Add Beacons</Label>
                    {addingBeacons ?
                    <div className='adding-beacons'>
                        <Button variant={"outline"} id='add-beacons' onClick={() => cancelAddBeacons()}>Cancel</Button>
                        <Button variant={"outline"} id='add-beacons' onClick={() => saveBeacons()}>Save</Button>
                    </div>
                        :
                    <Button variant={"outline"} id='add-beacons' onClick={() => setAddingBeacons(true)}>Add beacons</Button>
                    }
                </div>
            </div>
        </div>
    )
}