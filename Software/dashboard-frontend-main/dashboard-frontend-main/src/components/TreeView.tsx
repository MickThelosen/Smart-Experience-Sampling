'use client'

import React, { useState } from 'react';
import { ChevronRight, ChevronDown, Edit, Delete, Trash } from 'lucide-react';
import { Button } from "@/components/ui/button"
import { Location } from '@/types/location';

interface TreeNodeProps {
  location: Location
}

function TreeNode({location}: TreeNodeProps) {
  const [isOpen, setIsOpen] = useState(false);

  const toggleOpen = () => {
    setIsOpen(!isOpen);
  };

  return (
    <div>
      <div className="flex border-b" style={{
        justifyContent: 'space-between'
      }}>
        <div className='flex'>

        {location.child_locations?.length > 0 ? (
            <Button variant="ghost" size="sm" className="w-6 h-6 p-0" onClick={toggleOpen}>
            {isOpen ? <ChevronDown className="h-4 w-4" /> : <ChevronRight className="h-4 w-4" />}
          </Button>
        ) : (
            <div className="w-6 h-4" />
        )}
            <div style={{
                minHeight: '24px',
                padding: '2px 0px 2px 0px'
            }}>
            {location.name}
            </div>
        </div>
        <div>

        <div style={{
            display: 'flex',
            justifyContent: 'space-between'
        }}>
            <div style={{
                display: 'flex',
                gap: '20px'
            }}>
                <Edit style={{
                    color: 'orange',
                    cursor: 'pointer'
                }} />
                <Trash style={{
                    color: 'red',
                    cursor: 'pointer'
                }}/>
            </div>
                </div>
        </div>
      </div>
      {isOpen && location.child_locations.length > 0 && (
        <div className="ml-6 border-l pl-2">
          {location.child_locations.map((child) => (
            <TreeNode key={child.id} location={child} />
          ))}
        </div>
      )}
    </div>
  );
};

interface TreeViewProps {
    locations: Location[]
}

export function TreeView({locations} : TreeViewProps) {
    console.log(locations)
  return (
    <div className="p-4">
      {locations.map((location) => (
        <TreeNode key={location.id} location={location} />
      ))}
    </div>
  );
};