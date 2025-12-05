/* esp32-firmware
 * Copyright (C) 2022 Matthias Bolte <matthias@tinkerforge.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

import * as API from "../../ts/api";
import * as util from "../../ts/util";
import { h, Component } from "preact";
import { __ } from "../../ts/translation";
import { PageHeader } from "../../ts/components/page_header";
import { SubPage } from "../../ts/components/sub_page";
import { NavbarItem } from "../../ts/components/navbar_item";
import { Box } from "react-feather";


export function FlightControlNavbar() {
    return <NavbarItem name="flight_control" module="flight_control" title={__("flight_control.navbar.flight_control")} symbol={<Box />} />;
}

export class FlightControl extends Component<{}> {
    constructor() {
        super();

        util.addApiEventListener("flight_control/config", () => {
            /*
                Receive data from backend!
            */
            const fConfig = API.get("flight_control/config");
            console.log(fConfig);
        });

        util.addApiEventListener("flight_control/state", () => {
            /*
                Receive data from backend!
            */
            const fstate = API.get("flight_control/state");
            console.log(fstate);
        });        
    }
    onAltitudeSliderValueChanged = (event: Event) => {
        const target = event.target as HTMLInputElement;
        let config = {setAltitude: (parseFloat(target.value) / 100) };      
        API.save("flight_control/config", config, () => __("flight_control.script.save_config_failed"));        
    };
    
    onAltitudeSliderMouseUp = () => {
        const slider = document.getElementById("altitudeSlider") as HTMLInputElement;
        slider.value = "0";
        slider.dispatchEvent(new Event("input"));
    }

    render() {
        return (
            <SubPage name="flight_control">
                <PageHeader title={__("flight_control.content.flight_control")} />
                <input id="altitudeSlider" style="appearance: slider-vertical; writing-mode: bt-lr; width: 8px; height: 200px; transform: rotate(180deg);"  type="range" value="0" min="-100" max="100" onChange={this.onAltitudeSliderValueChanged} onMouseUp={this.onAltitudeSliderMouseUp}/>
            </SubPage>);
    }
}

export function init() {

   
}
