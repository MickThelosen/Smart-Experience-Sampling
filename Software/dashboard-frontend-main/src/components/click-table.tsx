'use client'

import { Click } from "@/types/click"
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table"
import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
} from "@/components/ui/tooltip"

interface ClickTableProps {
  clicks: Click[]
}

import { dateFormatOptions } from "../util/dateFormatOptions"

export default function ClickTable({ clicks }: ClickTableProps) {
  return (
    <TooltipProvider>
      <Table>
        <TableHeader>
          <TableRow>
            <TableHead>Click Time</TableHead>
            <TableHead>Clicker UID</TableHead>
            <TableHead>Beacons</TableHead>
          </TableRow>
        </TableHeader>
        <TableBody>
          {clicks.map((click) => (
            <TableRow key={click.id}>
              <TableCell>{new Date(click.click_time).toLocaleString('nl-NL', dateFormatOptions)}</TableCell>
              <TableCell>{click.clicker.uid}</TableCell>
              <TableCell>
                <div className="flex flex-wrap gap-2">
                  {click.click_beacons.map((beacon) => (
                    <Tooltip key={beacon.beacon.uid}>
                      <TooltipTrigger asChild>
                        <span className="bg-blue-100 text-blue-800 text-xs font-medium mr-2 px-2.5 py-0.5 rounded dark:bg-blue-900 dark:text-blue-300">
                          {beacon.beacon.uid} - Distance: {beacon.distance}
                        </span>
                      </TooltipTrigger>
                      <TooltipContent>
                        <p>Distance: {beacon.distance}</p>
                      </TooltipContent>
                    </Tooltip>
                  ))}
                </div>
              </TableCell>
            </TableRow>
          ))}
        </TableBody>
      </Table>
    </TooltipProvider>
  )
}

