import ned.IBidirectionalChannel;
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//
//--------FINAL_PROJECT----------
//  Saar Ben-Yochana - 313234155
//  Efi Korenfeld - 208838359
//-------------------------------

// Network1 topology: (6 nodes)
network Network1
{
    parameters:
        // Number of hosts in the network
        int num_hosts = default(6);					
        
        // Assigning Packet Error Rate on each link from ini file:
        double perlink1= default(0);
        double perlink2= default(0);
        double perlink3= default(0);
        double perlink4= default(0);
        double perlink5= default(0);
        double perlink6= default(0);
        double perlink7= default(0);
        double perlink8= default(0);
        double perlink9= default(0);
        double perlink10= default(0);
    	double perlink11 = default(0);
        double perlink12 = default(0);

       	// Statistics signals:
 		@statistic[costs](title="costs"; source="costs"; record=vector,stats; interpolationmode=none);
        @statistic[rewards](title="rewards"; source="rewards"; record=vector,stats; interpolationmode=none);
        @display("bgb=606,301");
        
    // Declaring the channel:
    types:
        channel Channel extends ned.DatarateChannel
        {
            delay = 0.00001s;           
        }

	// Creating host modules:	
    submodules:
        host: Host {
            @display("p=461,160;i=,green;t=Source");
        }
        host1: Host {
            @display("p=356,209");
        }
        host2: Host {
            @display("p=240,127");
        }
        host3: Host {
            @display("p=331,50");
        }
        host4: Host {
            @display("p=440,59");
        }
        host5: Host {
            @display("p=169,242;i=,#FF0080;t=Destination");
        }
        
    // Connecting the modules thorugh the channel and assigning PER:
    connections:
        host.interface++ <--> Channel { @display("ls=,1"); per=perlink1;} <--> host1.interface++;
        host.interface++ <--> Channel { @display("ls=,1"); per=perlink2;} <--> host2.interface++;
        host.interface++ <--> Channel { @display("ls=,1"); per=perlink3;} <--> host3.interface++;
        host.interface++ <--> Channel { @display("ls=,1"); per=perlink4;} <--> host4.interface++;
        host1.interface++ <--> Channel { @display("ls=,1"); per=perlink5;} <--> host2.interface++;
        host1.interface++ <--> Channel { @display("ls=,1"); per=perlink6;} <--> host3.interface++;
        host1.interface++ <--> Channel { @display("ls=,1"); per=perlink7;} <--> host4.interface++;
        host2.interface++ <--> Channel { @display("ls=,1"); per=perlink8;} <--> host3.interface++;
        host2.interface++ <--> Channel { @display("ls=,1"); per=perlink9;} <--> host4.interface++;
        host3.interface++ <--> Channel { @display("ls=,1"); per=perlink10;} <--> host4.interface++;
        host1.interface++ <--> Channel { @display("ls=,1"); per=perlink11;} <--> host5.interface++;
        host2.interface++ <--> Channel { @display("ls=,1"); per=perlink12;} <--> host5.interface++;
}


// Network2 topology: (60 nodes)
network Network2
{
    parameters:
        // Number of hosts in the network
        int num_hosts = default(60);	
        			
        // Assigning Packet Error Rate on each link from ini file:				
        double perlink1= default(0);
        double perlink2= default(0);
        double perlink3= default(0);
        double perlink4= default(0);
        double perlink5= default(0);
        double perlink6= default(0);
        double perlink7= default(0);
        double perlink8= default(0);
        double perlink9= default(0);
        double perlink10= default(0);
    	double perlink11 = default(0);
        double perlink12 = default(0);
        double perlink13= default(0);
        double perlink14= default(0);
        double perlink15= default(0);
        double perlink16= default(0);
        double perlink17= default(0);
        double perlink18= default(0);
        double perlink19= default(0);
        double perlink20= default(0);
        double perlink21= default(0);
        double perlink22 = default(0);
        double perlink23= default(0);
        double perlink24= default(0);
        double perlink25= default(0);
        double perlink26= default(0);
        double perlink27= default(0);
        double perlink28= default(0);
        double perlink29= default(0);
        double perlink30= default(0);
    	double perlink31 = default(0);
        double perlink32 = default(0);
        double perlink33= default(0);
        double perlink34= default(0);
        double perlink35= default(0);
        double perlink36= default(0);
        double perlink37= default(0);
        double perlink38= default(0);
        double perlink39= default(0);
        double perlink40= default(0);
    	double perlink41 = default(0);
        double perlink42 = default(0);
        double perlink43= default(0);
        double perlink44= default(0);
        double perlink45= default(0);
        double perlink46= default(0);
        double perlink47= default(0);
        double perlink48= default(0);
        double perlink49= default(0);
        double perlink50= default(0);
    	double perlink51 = default(0);
        double perlink52 = default(0);
        double perlink53= default(0);
        double perlink54= default(0);
        double perlink55= default(0);
        double perlink56= default(0);
        double perlink57= default(0);
        double perlink58= default(0);
        double perlink59= default(0);
        double perlink60= default(0);
    	double perlink61 = default(0);
        double perlink62 = default(0);
        double perlink63= default(0);
        double perlink64= default(0);
        double perlink65= default(0);
        double perlink66= default(0);
        double perlink67= default(0);
        double perlink68= default(0);
        double perlink69= default(0);
        double perlink70= default(0);
    	double perlink71 = default(0);
        double perlink72 = default(0);
        double perlink73= default(0);
        double perlink74= default(0);
        double perlink75= default(0);
        double perlink76= default(0);
        double perlink77= default(0);
        double perlink78= default(0);
        double perlink79= default(0);
        double perlink80= default(0);
    	double perlink81 = default(0);
        double perlink82 = default(0);
        double perlink83= default(0);
        double perlink84= default(0);
        double perlink85= default(0);
        double perlink86= default(0);
        double perlink87= default(0);
        double perlink88= default(0);
        double perlink89= default(0);
        double perlink90= default(0);

		// Statistics signals:
		@statistic[costs](title="costs"; source="costs"; record=vector,stats; interpolationmode=none);
        @statistic[rewards](title="rewards"; source="rewards"; record=vector,stats; interpolationmode=none);
        @display("bgb=2386.28,1308.0599");
        
    // Declaring the channel:
    types:
        channel Channel extends ned.DatarateChannel
        {
            delay = 0.00001s;
        }
        
	// Creating host modules:
    submodules:
        host: Host {
            @display("p=1037.66,922.74;t=Source;is=n;i=,#00FF40");
        }
        host1: Host {
            @display("p=1213.4199,853.44995");
        }
        host2: Host {
            @display("p=1368.9,964.99");
        }
        host3: Host {
            @display("p=1286.09,1066.39");
        }
        host4: Host {
            @display("p=1076.53,1025.83");
        }
        host5: Host {
            @display("p=921.05,823.02997");
        }
        host6: Host {
            @display("p=774.01996,878.8");
        }
        host7: Host {
            @display("p=708.11,689.51996");
        }
        host8: Host {
            @display("p=823.02997,633.75");
        }
        host9: Host {
            @display("p=951.47,643.88995");
        }
        host10: Host {
            @display("p=1242.15,725.01");
        }
        host11: Host {
            @display("p=1127.23,645.57996");
        }
        host12: Host {
            @display("p=1203.28,490.09998");
        }
        host13: Host {
            @display("p=1324.96,512.07");
        }
        host14: Host {
            @display("p=1450.02,642.2");
        }
        host15: Host {
            @display("p=1487.2,878.8");
        }
        host16: Host {
            @display("p=1553.11,657.41");
        }
        host17: Host {
            @display("p=1619.02,577.98");
        }
        host18: Host {
            @display("p=1619.02,726.69995");
        }
        host19: Host {
            @display("p=1588.6,878.8");
        }
        host20: Host {
            @display("p=1404.39,1177.9299");
        }
        host21: Host {
            @display("p=1505.7899,1066.39");
        }
        host22: Host {
            @display("p=1286.09,988.64996");
        }
        host23: Host {
            @display("p=1147.51,1066.39");
        }
        host24: Host {
            @display("p=1147.51,1233.7");
        }
        host25: Host {
            @display("p=936.25995,1122.1599");
        }
        host26: Host {
            @display("p=951.47,1220.1799");
        }
        host27: Host {
            @display("p=785.85,1198.21");
        }
        host28: Host {
            @display("p=632.06,1081.6");
        }
        host29: Host {
            @display("p=757.12,1025.83");
        }
        host30: Host {
            @display("p=1439.88,456.3");
        }
        host31: Host {
            @display("p=1571.7,446.16");
        }
        host32: Host {
            @display("p=1487.2,321.1");
        }
        host33: Host {
            @display("p=1368.9,236.59999");
        }
        host34: Host {
            @display("p=1352,365.03998");
        }
        host35: Host {
            @display("p=1176.24,202.79999");
        }
        host36: Host {
            @display("p=1061.32,331.24");
        }
        host37: Host {
            @display("p=907.52997,390.38998");
        }
        host38: Host {
            @display("p=730.07996,331.24");
        }
        host39: Host {
            @display("p=936.25995,201.11");
        }
        host40: Host {
            @display("p=500.24,545.87");
        }
        host41: Host {
            @display("p=434.33,725.01");
        }
        host42: Host {
            @display("p=544.18,932.88");
        }
        host43: Host {
            @display("p=544.18,789.23");
        }
        host44: Host {
            @display("p=596.57,601.64");
        }
        host45: Host {
            @display("p=677.69,851.75995");
        }
        host46: Host {
            @display("p=841.62,988.64996");
        }
        host47: Host {
            @display("p=951.47,988.64996");
        }
        host48: Host {
            @display("p=1037.66,789.23");
        }
        host49: Host {
            @display("p=851.75995,767.26");
        }
        host50: Host {
            @display("p=1147.51,789.23");
        }
        host51: Host {
            @display("p=1324.96,845");
        }
        host52: Host {
            @display("p=1487.2,745.29");
        }
        host53: Host {
            @display("p=1450.02,577.98");
        }
        host54: Host {
            @display("p=1286.09,633.75");
        }
        host55: Host {
            @display("p=1176.24,331.24");
        }
        host56: Host {
            @display("p=936.25995,300.82");
        }
        host57: Host {
            @display("p=806.13,420.81");
        }
        host58: Host {
            @display("p=951.47,512.07");
        }
        host59: Host {
            @display("p=1076.53,476.58;i=,#FF0080;t=Destination");
        }
        
    // Connecting the modules thorugh the channel and assigning PER:
    connections:
        host.interface++ <--> Channel { @display("ls=,1"); per=perlink1;} <--> host1.interface++;
        host1.interface++ <--> Channel { @display("ls=,1"); per=perlink2;} <--> host2.interface++;
        host5.interface++ <--> Channel { @display("ls=,1"); per=perlink3;} <--> host6.interface++;
        host6.interface++ <--> Channel { @display("ls=,1"); per=perlink4;} <--> host7.interface++;
        host7.interface++ <--> Channel { @display("ls=,1"); per=perlink5;} <--> host8.interface++;
        host8.interface++ <--> Channel { @display("ls=,1"); per=perlink6;} <--> host9.interface++;
        host9.interface++ <--> Channel { @display("ls=,1"); per=perlink7;} <--> host5.interface++;
        host2.interface++ <--> Channel { @display("ls=,1"); per=perlink8;} <--> host3.interface++;
        host3.interface++ <--> Channel { @display("ls=,1"); per=perlink9;} <--> host4.interface++;
        host4.interface++ <--> Channel { @display("ls=,1"); per=perlink10;} <--> host.interface++;
        host10.interface++ <--> Channel { @display("ls=,1"); per=perlink11;} <--> host11.interface++;
        host11.interface++ <--> Channel { @display("ls=,1"); per=perlink12;} <--> host12.interface++;
        host12.interface++ <--> Channel { @display("ls=,1"); per=perlink13;} <--> host13.interface++;
        host14.interface++ <--> Channel { @display("ls=,1"); per=perlink14;} <--> host10.interface++;
        host13.interface++ <--> Channel { @display("ls=,1"); per=perlink15;} <--> host14.interface++;
        host15.interface++ <--> Channel { @display("ls=,1"); per=perlink16;} <--> host16.interface++;
        host16.interface++ <--> Channel { @display("ls=,1"); per=perlink17;} <--> host17.interface++;
        host17.interface++ <--> Channel { @display("ls=,1"); per=perlink18;} <--> host18.interface++;
        host19.interface++ <--> Channel { @display("ls=,1"); per=perlink19;} <--> host15.interface++;
        host18.interface++ <--> Channel { @display("ls=,1"); per=perlink20;} <--> host19.interface++;
        host20.interface++ <--> Channel { @display("ls=,1"); per=perlink21;} <--> host21.interface++;
        host21.interface++ <--> Channel { @display("ls=,1"); per=perlink22;} <--> host22.interface++;
        host22.interface++ <--> Channel { @display("ls=,1"); per=perlink23;} <--> host23.interface++;
        host24.interface++ <--> Channel { @display("ls=,1"); per=perlink24;} <--> host20.interface++;
        host23.interface++ <--> Channel { @display("ls=,1"); per=perlink25;} <--> host24.interface++;
        host25.interface++ <--> Channel { @display("ls=,1"); per=perlink26;} <--> host26.interface++;
        host26.interface++ <--> Channel { @display("ls=,1"); per=perlink27;} <--> host27.interface++;
        host27.interface++ <--> Channel { @display("ls=,1"); per=perlink27;} <--> host28.interface++;
        host29.interface++ <--> Channel { @display("ls=,1"); per=perlink28;} <--> host25.interface++;
        host28.interface++ <--> Channel { @display("ls=,1"); per=perlink29;} <--> host29.interface++;
        host30.interface++ <--> Channel { @display("ls=,1"); per=perlink30;} <--> host31.interface++;
        host31.interface++ <--> Channel { @display("ls=,1"); per=perlink31;} <--> host32.interface++;
        host32.interface++ <--> Channel { @display("ls=,1"); per=perlink32;} <--> host33.interface++;
        host34.interface++ <--> Channel { @display("ls=,1"); per=perlink33;} <--> host30.interface++;
        host33.interface++ <--> Channel { @display("ls=,1"); per=perlink34;} <--> host34.interface++;
        host35.interface++ <--> Channel { @display("ls=,1"); per=perlink35;} <--> host36.interface++;
        host36.interface++ <--> Channel { @display("ls=,1"); per=perlink36;} <--> host37.interface++;
        host37.interface++ <--> Channel { @display("ls=,1"); per=perlink37;} <--> host38.interface++;
        host39.interface++ <--> Channel { @display("ls=,1"); per=perlink38;} <--> host35.interface++;
        host38.interface++ <--> Channel { @display("ls=,1"); per=perlink39;} <--> host39.interface++;
        host40.interface++ <--> Channel { @display("ls=,1"); per=perlink40;} <--> host41.interface++;
        host41.interface++ <--> Channel { @display("ls=,1"); per=perlink41;} <--> host42.interface++;
        host42.interface++ <--> Channel { @display("ls=,1"); per=perlink42;} <--> host43.interface++;
        host44.interface++ <--> Channel { @display("ls=,1"); per=perlink43;} <--> host40.interface++;
        host43.interface++ <--> Channel { @display("ls=,1"); per=perlink44;} <--> host44.interface++;
        host45.interface++ <--> Channel { @display("ls=,1"); per=perlink45;} <--> host46.interface++;
        host46.interface++ <--> Channel { @display("ls=,1"); per=perlink46;} <--> host47.interface++;
        host47.interface++ <--> Channel { @display("ls=,1"); per=perlink47;} <--> host48.interface++;
        host49.interface++ <--> Channel { @display("ls=,1"); per=perlink48;} <--> host45.interface++;
        host48.interface++ <--> Channel { @display("ls=,1"); per=perlink49;} <--> host49.interface++;
        host.interface++ <--> Channel { @display("ls=,1"); per=perlink50;} <--> host5.interface++;
        host2.interface++ <--> Channel { @display("ls=,1"); per=perlink51;} <--> host15.interface++;
        host21.interface++ <--> Channel { @display("ls=,1"); per=perlink52;} <--> host19.interface++;
        host26.interface++ <--> Channel { @display("ls=,1"); per=perlink53;} <--> host24.interface++;
        host3.interface++ <--> Channel { @display("ls=,1"); per=perlink54;} <--> host20.interface++;
        host4.interface++ <--> Channel { @display("ls=,1"); per=perlink55;} <--> host25.interface++;
        host28.interface++ <--> Channel { @display("ls=,1"); per=perlink56;} <--> host42.interface++;
        host17.interface++ <--> Channel { @display("ls=,1"); per=perlink57;} <--> host31.interface++;
        host35.interface++ <--> Channel { @display("ls=,1"); per=perlink58;} <--> host33.interface++;
        host38.interface++ <--> Channel { @display("ls=,1"); per=perlink59;} <--> host40.interface++;
        host1.interface++ <--> Channel { @display("ls=,1"); per=perlink60;} <--> host10.interface++;
        host14.interface++ <--> Channel { @display("ls=,1"); per=perlink61;} <--> host16.interface++;
        host13.interface++ <--> Channel { @display("ls=,1"); per=perlink62;} <--> host32.interface++;
        host12.interface++ <--> Channel { @display("ls=,1"); per=perlink63;} <--> host36.interface++;
        host9.interface++ <--> Channel { @display("ls=,1"); per=perlink64;} <--> host11.interface++;
        host51.interface++ <--> Channel { @display("ls=,1"); per=perlink65;} <--> host52.interface++;
        host50.interface++ <--> Channel { @display("ls=,1"); per=perlink66;} <--> host51.interface++;
        host53.interface++ <--> Channel { @display("ls=,1"); per=perlink67;} <--> host54.interface++;
        host52.interface++ <--> Channel { @display("ls=,1"); per=perlink68;} <--> host53.interface++;
        host54.interface++ <--> Channel { @display("ls=,1"); per=perlink69;} <--> host50.interface++;
        host56.interface++ <--> Channel { @display("ls=,1"); per=perlink70;} <--> host57.interface++;
        host55.interface++ <--> Channel { @display("ls=,1"); per=perlink71;} <--> host56.interface++;
        host58.interface++ <--> Channel { @display("ls=,1"); per=perlink72;} <--> host59.interface++;
        host57.interface++ <--> Channel { @display("ls=,1"); per=perlink73;} <--> host58.interface++;
        host59.interface++ <--> Channel { @display("ls=,1"); per=perlink74;} <--> host55.interface++;
        host6.interface++ <--> Channel { @display("ls=,1"); per=perlink75;} <--> host29.interface++;
        host7.interface++ <--> Channel { @display("ls=,1"); per=perlink76;} <--> host41.interface++;
        host39.interface++ <--> Channel { @display("ls=,1"); per=perlink77;} <--> host56.interface++;
        host57.interface++ <--> Channel { @display("ls=,1"); per=perlink78;} <--> host44.interface++;
        host55.interface++ <--> Channel { @display("ls=,1"); per=perlink79;} <--> host34.interface++;
        host37.interface++ <--> Channel { @display("ls=,1"); per=perlink80;} <--> host8.interface++;
        host43.interface++ <--> Channel { @display("ls=,1"); per=perlink81;} <--> host45.interface++;
        host58.interface++ <--> Channel { @display("ls=,1"); per=perlink82;} <--> host49.interface++;
        host47.interface++ <--> Channel { @display("ls=,1"); per=perlink83;} <--> host23.interface++;
        host27.interface++ <--> Channel { @display("ls=,1"); per=perlink84;} <--> host46.interface++;
        host48.interface++ <--> Channel { @display("ls=,1"); per=perlink85;} <--> host50.interface++;
        host51.interface++ <--> Channel { @display("ls=,1"); per=perlink86;} <--> host22.interface++;
        host18.interface++ <--> Channel { @display("ls=,1"); per=perlink87;} <--> host52.interface++;
        host30.interface++ <--> Channel { @display("ls=,1"); per=perlink88;} <--> host53.interface++;
        host59.interface++ <--> Channel { @display("ls=,1"); per=perlink89;} <--> host54.interface++;
}

