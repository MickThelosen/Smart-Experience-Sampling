'use client'

import { getRootLocations } from "@/api/location/getRootLocations";
import FAB from "@/components/FAB";
import { TreeView } from "@/components/TreeView";
import { Location } from "@/types/location";
import { Plus } from "lucide-react";
import { useEffect, useState } from "react";
import CreateLocationModal from "./createLocationModal";
import saveLocation from "@/api/location/createLocation";

export default function LocationsPage() {
    const [locations, setLocations] = useState<Location[]>([])
    const [createLocationOpen, setCreateLocationOpen] = useState<boolean>(false)
    useEffect(() => {
        async function getLocations() {
          
          let locations = await getRootLocations()
          setLocations(locations)
          console.log(locations)
        }
    
        getLocations()
    }, [])

    const createLocation = () => {
        setCreateLocationOpen(true)
    }

    const submit = (name: string, parentId: string | null, image?: File) => {
        saveLocation(name, parentId, image)
    }

    return (

<main className="container mx-auto" style={{padding: '40px'}}>
    <CreateLocationModal
        isOpen={createLocationOpen}
        onClose={() => setCreateLocationOpen(false)}
        onCreateLocation={submit}
        locations={locations}/>
    <h1 className="text-2xl font-bold mb-4">Location Tree View</h1>
    {locations.length != 0 ? <TreeView locations={locations} /> : <div>No locations found!</div>}
    
    <FAB variant={'default'} size={'icon'} icon={Plus} onClick={createLocation} />
</main>
    )
}