import { MapBeacon } from "@/types/beacon"

interface MapProps {
    image: string
    beacons: MapBeacon[]
    onMapClick: (x: number, y: number) => void
}

export default function MapCore({image, beacons, onMapClick} : MapProps) {
    
}