'use client'

import FAB from "@/components/FAB";
import { Button } from "@/components/ui/button";
import { Plus } from "lucide-react";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { useEffect, useState } from "react";
import { getRootLocations } from "@/api/location/getRootLocations";
import { flattenLocations } from "../locations/createLocationModal";
import { Location } from "@/types/location";

import './research.css'
import ResearchModal from "./createResearchModal";

import { FormValues } from "./createResearchModal";
import createResearch from "@/api/createResearch";
import { Research } from "@/types/research";
import getResearchesFromLocation from "@/api/getResearchesFromLocation";


export default function ResearchPage() {
    const [locations, setLocations] = useState<Location[]>([])
    const [researches, setResearches] = useState<Research[]>([])
    const [id, setId] = useState<string | null>("")
    const flattenedLocations = flattenLocations(locations)
    const [researchModelOpen, setResearchModelOpen] = useState<boolean>(false)

    useEffect(() => {
        async function getLocations() {
          
          let locations = await getRootLocations()
          setLocations(locations)
        }
    
        getLocations()
    }, [])

    async function getResearches(id: string) {
        if (id === "none") {
            setResearches([])
            return
        }  
        let researches = await getResearchesFromLocation(id)
        setResearches(researches)
        console.log(researches)
      }

    const handleSubmit = (values: FormValues) => {
        createResearch(values.question, values.startDate.toISOString(), values.endDate.toISOString(), id || "")
    }

    return (
<>
<ResearchModal open={researchModelOpen} setOpen={setResearchModelOpen} handleSubmit={handleSubmit}/>
<div className="research_settings">
    <p>Select location:</p>
<Select onValueChange={(value) => {
    setId(value === "none" ? null : value)
    getResearches(value)
    }} value={id || "none"}>
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
<div style={{
    width: '100%',
    height: '180px',
    boxShadow: 'lightGray 0px 0px 20px 10px inset',
    display: 'flex'
}}>
    <div className='add_research_wrapper'>

    <div className="add_research" onClick={() => id != "" && setResearchModelOpen(true)}>+</div>
    </div>
    {
        researches.map((research, index) => (
            <>
            <div key={research.id} className="research">
                <p style={{
                    fontWeight: '600'
                }}>Research:</p>
                <p>
                    {research.question}
                </p>
                <p>Start date:</p>
                <p>{research.startTime.toLocaleDateString('nl-NL', { year:"numeric", month:"short", day:"numeric"})}</p>
                <p>End date:</p>
                <p>{research.startTime.toLocaleDateString('nl-NL', { year:"numeric", month:"short", day:"numeric"})}</p>

            </div>
            <div className='add_research_wrapper'>
                <div className="add_research" onClick={() => setResearchModelOpen(true)}>+</div>
            </div>
            </>

        ))
    }
    {/* <div style={{
        height: '100%',
        width: '10%',
        backgroundColor: '#d4d4d8',
        borderRadius: '10px'
    }}>Research</div>
    <div style={{
        height: '100%',
        width: '5%',
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center'
    }}>+</div>
    <div style={{
        height: '100%',
        width: '10%',
        backgroundColor: 'gray',
        borderRadius: '10px'
    }}>Research</div>
    <div style={{
        height: '100%',
        width: '5%',
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center'
    }}>+</div>
    <div style={{
        height: '100%',
        width: '10%',
        backgroundColor: 'gray',
        borderRadius: '10px'
    }}>Research</div> */}
</div>
        {/* floating action button: */}
        <FAB variant={'default'} size={'icon'} icon={Plus} onClick={() => id != "" && setResearchModelOpen(true)} />
        </>
    )
}