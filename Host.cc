/*
 * --------FINAL_PROJECT----------
 * Saar Ben-Yochana - *********
 * Efi Korenfeld - *********
 * -------------------------------
 */

// Declaring Defines:
#define MAXPACKETS 2000
#define SOURCE 0
#define DATA 0
#define ACK 1
#define NACK 2
#define LACK 3
#define LCFM 4

#include "Host.h"
using namespace std;

Define_Module(Host);

Host::Host(){}

Host::~Host() {
    // Delete the last data message saved
    delete(data_message);
    // Cancel and delete the generator message
    cancelAndDelete(generate);
}

void Host::initialize(){
    // Get parameters from ned file
    R = par("R");
    TARGET = par("TARGET");
    net_type = par("net");
    OPT = par("OPT");
    source_path = par("source_path").stdstringValue();
    // Find self host index
    host_index = this->getId()-2;

    // Register statistic signals and vectors
    costs_Signal = registerSignal("costs");
    rewards_Signal = registerSignal("rewards");
    distance_vector.setName("distance vector");
    sequences_vec.setName("Sequences vector");

    // According to TSOR, destination is initialized with V = -R
    if (host_index == TARGET){
        V = -R;
    }

    //Initialize neighbors tables:
    int num_connections = gateSize("interface$o");
    int neighbor;
    for (int i=0;i<num_connections;i++){
      cGate* g = gate("interface$o",i);
      cGate* next = g->getNextGate();
      neighbor = next->getOwnerModule()->getId()-2;
      // Beta params are (alpha,beta) = (1,1)
      beta_params[neighbor] = pair<int,int>(1,1);
      // Neighbor distances are 0 except for destination who is -R
      neighbors_distances[neighbor] = neighbor == TARGET ? -R : 0;
      // Mapping from neighbor id to out gate
      neighbor_gates[neighbor] = i;
    }
    // View initialized tables
    printTables();

    if (host_index == SOURCE){
        // Schedule the generate to create first message:
        generate = new cMessage("generate");
        simtime_t interval = par("sendingInterval");
        scheduleAt(0.0 + interval, generate);
    }
}




void Host::handleMessage(cMessage *msg){
    if (msg->getArrivalGate() == gate("control")){
        // If recieved message in control gate, collect statistics
        pkt *rcvd_pkt = check_and_cast<pkt *>(msg);
        distance_vector.record(V);
        sequences_vec.record(rcvd_pkt->getSequence_number());
        delete(msg);
    }
    else{
        if (msg->isSelfMessage()){
            // Self message means it is the generator, create new data message and broadcast
            pkt *msg = generateMessage(DATA,TARGET,"Data Message",msg_num);
            broadcast(msg);
            delete(msg);
            // Schedule next generator message
            if (msg_num <= MAXPACKETS){
                simtime_t interval = par("sendingInterval");
                scheduleAt(simTime() + interval, generate);
            }
        }
        else{
            // Cast message, check type and call the appropriate handle function
            pkt *rcvd_pkt = check_and_cast<pkt *>(msg);
            switch(rcvd_pkt->getType()){
                case DATA:
                    handleDataMsg(rcvd_pkt);
                    break;
                case ACK:
                    handleAckNackMsg(rcvd_pkt,"ACK");
                    break;
                case NACK:
                    handleAckNackMsg(rcvd_pkt,"NACK");
                    break;
                case LACK:
                    handleLackMsg(rcvd_pkt);
                    break;
                case LCFM:
                    handleLcfmMsg(rcvd_pkt);
                    break;
            }
        }
    }
}



void Host::handleDataMsg(pkt* msg)
{
    EV<< "------------------------------------------------------"<<endl;
    EV<< getFullPath() <<" Handling Data Message: " << msg->getSequence_number() <<endl;

    // Create lack1:
    int lack_dest = msg->getSrc();
    pkt *lack = generateMessage(LACK, lack_dest, "LACK1 Message",msg->getSequence_number());
    lack->setLack_type(1);

    if(msg->hasBitError() || msg->getTtl() == 0){
        // If message dropped, we send corrupted Lack (for simulation purposes)
        EV<< "Data Message Dropped"<< endl;
        delete(msg);
        msg=nullptr;
        lack->setBitError(true);
    }
    else{
        // If data message successfully received, delete last saved message and save new
        if(data_message){
            delete(data_message);
            data_message=nullptr;
        }
        data_message = msg->dup();
    }

    if (msg && msg->getDst() == host_index){
            EV<< "Message Successfully recieved at destination, sending ack"<<endl;

            // If message successfully received at destination, send ack and record statistics
            pkt *ack = generateMessage(ACK, SOURCE, "ACK Message",msg->getSequence_number());
            broadcast(ack);
            delete(ack);
            msg->setHop_count(msg->getHop_count()+1);
            emit(costs_Signal,msg->getHop_count());
            emit(rewards_Signal,R);
            pkt* control = new pkt("Statistics Signal");
            control->setSequence_number(msg->getSequence_number());
            sendDirect(control,getModuleByPath(source_path.c_str()),"control");
    }

    EV << "Sending Lack type 1 Message: "<<host_index << " -> "<< lack_dest<<endl;

    // Respond with lack to the sender
    sendDelayed(lack,0.000001,"interface$o",neighbor_gates[lack_dest]);
    delete(msg);
    EV<< "------------------------------------------------------"<<endl;
}


void Host::handleLackMsg(pkt* lack){
    EV<< "------------------------------------------------------"<<endl;

    // Handling lack1 (lack received in response to data message)
    if(lack->getLack_type() == 1){
        EV<< getFullPath() <<" Handling Lack type 1 Message Sequence: " << lack->getSequence_number() <<endl;
        lack_recieved++;
        EV <<host_index<<": Lack1 count: "<< lack_recieved <<endl;

        // If received lack1 successfully, record the advertised distance, pick randomly if there are ties
        if(!(lack->hasBitError())){
            // Update alpha if lack successful
            beta_params[lack->getSrc()].first += 1;
            if(lack_distances.count(lack->getVn_tag())){
                double p = uniform(0,1);
                if (p<0.5){
                    lack_distances[lack->getVn_tag()] = lack->getSrc();
                }
            }
            else{
            lack_distances[lack->getVn_tag()] = lack->getSrc();
            }

        }
        else{
            EV<< "Lack1 Dropped"<< endl;
            // Update Beta if lack1 dropped
            beta_params[lack->getSrc()].second += 1;
        }

        // If recieved all Lack1
        if (lack_recieved == neighbor_gates.size()){
            EV<< "Received all Lack1"<< endl;

            // Check all advertised distances and pick neighbor with minimal distance
            if (lack_distances.size()){
                map<double, int>::iterator itr;
                for (itr = lack_distances.begin(); itr != lack_distances.end(); ++itr) {
                   EV<< getFullPath() <<" Neighbor: "<< itr->second <<" Dist: " << itr->first <<endl;
                 }

                int selected_neighbor =lack_distances.begin()->second;
                // Send LCFM to the selected neighbor
                if (selected_neighbor != TARGET){
                    pkt *lcfm = generateMessage(LCFM,selected_neighbor,"LCFM Message",lack->getSequence_number());
                    sendDelayed(lcfm,0.000001,"interface$o",neighbor_gates[selected_neighbor]);
                    printTables();
                }
                else{
                    // Special case- if the selected neighbor is the destination it means we received a
                    // lack1 from him but the data message was not successfully received, so we broadcast it again
                    if(last_msg_stat != data_message->getSequence_number() ){
                        broadcast(data_message);
                    }
                }
            }

            // If no available neighbors
            else{
                // Drop message , generate Nack and record statistics
                EV << "No available neighbor, drop Packet and generate Nack if not source"<<endl;
                pkt* control = new pkt("Statistics Signal");

                if(host_index != SOURCE){
                    pkt *nack = generateMessage(NACK, SOURCE, "NACK Message",lack->getSequence_number());
                    broadcast(nack);
                    delete(nack);
                    emit(costs_Signal,data_message->getHop_count());
                    control->setSequence_number(data_message->getSequence_number());
                }
                else{
                    // If the SOURCE has no available neighbors it just records statistics
                    emit(costs_Signal,0);
                    control->setSequence_number(msg_num-1);
                }
                emit(rewards_Signal,0);
                sendDirect(control,getModuleByPath(source_path.c_str()),"control");

            }
            // Clear lack counter and table
            lack_recieved = 0;
            lack_distances.clear();

        }
    }

    // If received lack2 (response to ACK/NACK messages)
    else{
        EV<< getFullPath() <<" Handling Lack type 2 Message Sequence: " << lack->getSequence_number() <<endl;
       if(!(lack->hasBitError())){
           // If lack2 recieved successfully update link alpha
           EV<< "Lack2 Recieved"<< endl;
           beta_params[lack->getSrc()].first += 1;
       }
       else{
           EV<< "Lack2 Dropped"<< endl;
           // If lack2 dropped update link beta
           beta_params[lack->getSrc()].second += 1;
       }
       EV <<"New Beta Params for: "<<lack->getSrc()<< ": alpha="<<beta_params[lack->getSrc()].first<<", beta="<<beta_params[lack->getSrc()].second<<endl;
    }
    delete(lack);
}


void Host::handleLcfmMsg(pkt* lcfm){
    // Handling LCFM- update hop count of data message and broadcast it
    EV <<"IN LCFM: " << data_message <<"Message: "<< data_message->getSequence_number() <<endl;
    delete(lcfm);
    data_message->setHop_count(data_message->getHop_count()+1);
    broadcast(data_message);
}

void Host::handleAckNackMsg(pkt* ack,string name){

    // Maintain last Acked (or Nacked) message sequence number
    last_msg_stat = ack->getSequence_number();

    EV<< "------------------------------------------------------"<<endl;
    EV<< getFullPath() <<" Handling "<<name<< " Message: " << ack->getSequence_number() <<endl;

    // First create Lack2
    int lack_dest = ack->getSrc();
    pkt *lack = generateMessage(LACK, lack_dest, "LACK2 Message",ack->getSequence_number());
    lack->setLack_type(2);

    // If ack is dropped, send corrupted lack2
    if(ack->hasBitError() || ack->getTtl() == 0){
        EV<< name<<" Dropped"<< endl;
        lack->setBitError(true);
        delete(ack);
        EV << "Sending Lack type 2 Message: "<<host_index << " -> "<< lack_dest<<endl;
        sendDelayed(lack,0.000001,"interface$o",neighbor_gates[lack_dest]);
    }
    else{
        // If ack successfully received, first reset lack11 counter and table, and send lack2
        lack_recieved = 0;
        lack_distances.clear();

        EV << "Sending Lack type 2 Message: "<<host_index << " -> "<< lack_dest<<endl;
        sendDelayed(lack,0.000001,"interface$o",neighbor_gates[lack_dest]);

        // Then, if the distance advertised in ACK is different then the distance we maintained (or we are connected to the target and then we always update)
        if (neighbors_distances[ack->getSrc()] != ack->getVn_tag() || ack->getSrc()==TARGET){
            // Update the neighbor distance
            neighbors_distances[ack->getSrc()] = ack->getVn_tag();
            if (host_index!= TARGET){

                // Except for target , update self distance based on beta realization and neighbors advertised distances
                double best_dist =0.0;
                EV << "Updating V, old: "<< V<<endl;
                map<int, double>::iterator itr;
                for (itr = neighbors_distances.begin(); itr != neighbors_distances.end(); ++itr) {
                    double beta_realization;
                    // If using OPT, the link probabilities are known
                    if(OPT){
                        int gate_index = neighbor_gates[itr->first];
                        cDatarateChannel * c = check_and_cast<cDatarateChannel *>(gate("interface$o",gate_index)->getTransmissionChannel());
                        EV << "PER: "<< c->getPacketErrorRate() <<endl;
                        double succ = 1 - c->getPacketErrorRate();
                        beta_realization = pow(succ,2);
                        EV <<"Realization: "<<beta_realization<<endl;
                    }
                    // When using TSOR, generate a realization based on saved (alpha,beta) for the link
                    else{
                        beta_realization= beta(beta_params[itr->first].first,beta_params[itr->first].second);
                    }
                    // Calculate distances and update to the minimal
                    double neighbor_dist = itr->second;
                    EV << "Neighbor: "<< itr->first<<" beta params, alpha= "<<beta_params[itr->first].first <<" beta= "<<beta_params[itr->first].second<<endl;
                    EV<< "Caluculating, beta realization: "<<beta_realization<<" neighbor distance: "<<neighbor_dist<<" overall: "<<beta_realization * neighbor_dist<<endl;
                    best_dist = std::min(best_dist, beta_realization * neighbor_dist);
                }
                // New distance is neighbors best distance + cost (=1)
                V = std::min(0.0,best_dist + 1);
                EV << "New V: "<< V<<endl;

            }
            // Broadcast the ACK
            ack->setVn_tag(V);
            broadcast(ack);
            delete(ack);

        }
        else{
            // If neighbor distance is the same as maintained in table, continue
            EV<< "Neighbor distance not changed"<<endl;
            delete(ack);
        }
    }
}


void Host::broadcast(pkt* msg){
    int num_connections = gateSize("interface$o");
    msg->setTtl(msg->getTtl()-1);
    // If TTL reaches 0 do not broadcast message
    if(msg->getTtl()==0){
        // If it was data message, record statistics
        if(msg->getType() == DATA){
            emit(costs_Signal,data_message->getHop_count()+1);
            emit(rewards_Signal,0);
            pkt* control = new pkt("Statistics Signal");
            control->setSequence_number(data_message->getSequence_number());
            sendDirect(control,getModuleByPath(source_path.c_str()),"control");

            // Generate NACK
            pkt *nack = generateMessage(NACK, SOURCE, "NACK Message",msg->getSequence_number());
            broadcast(nack);
            delete(nack);
        }
        return;
    }

    // Send message from all available gates
    for (int i=0;i<num_connections;i++){
        msg->setSrc(host_index);
        sendDelayed(msg->dup(),0.000001,"interface$o",i);
    }

}


pkt *Host::generateMessage(int type,int dest, string msg_name, int seq_num)
{
    // If generating data message, update sequence counter
    if (type == DATA){
        msg_name = msg_name + to_string(msg_num);
        msg_num++;
    }
    else{
        msg_name = msg_name + to_string(seq_num);
    }

    // Set all message fields
    pkt *msg = new pkt(msg_name.c_str());
    msg->setSequence_number(seq_num);       // Set sequence number
    msg->setHop_count(0);                   // Set hop count to 0
    msg->setSrc(host_index);                // Set source as the sending host
    msg->setTtl(par("ttl"));                // Set TTL
    msg->setDst(dest);                      // Set destination as specified
    msg->setType(type);                     // Set type of message
    msg->setVn_tag(V);                      // Add self distance to message

    // Special case for network2, due to heavy computations, we set all non-data messages with TTL=13
    if (net_type ==2){
        if (type != DATA){
            msg->setTtl(13);
        }
    }
    return msg;
}


void Host::printTables()
{
    // Print all maintained tables: neighbor_distances,beta_params and neighbor_gates
    EV<< "---------------------------------------------------------------"<<endl;
    EV<< getFullPath() <<" Beta table: "<<endl;

    map<int, pair<int,int>>::iterator itr;
    for (itr = beta_params.begin(); itr != beta_params.end(); ++itr) {
        EV<< getFullPath() <<" Neighbor: "<< itr->first <<" alpha: " <<itr->second.first <<" Beta: " <<itr->second.second <<endl;
    }
    EV<< "---------------------------------------------------------------"<<endl;
    EV<< getFullPath() <<" Neighbors distances table: "<<endl;
    map<int, double>::iterator itr2;
    for (itr2 = neighbors_distances.begin(); itr2 != neighbors_distances.end(); ++itr2) {
        EV<< getFullPath() <<" Neighbor: "<< itr2->first <<" Distance: " <<itr2->second <<endl;
    }
    EV<< "---------------------------------------------------------------"<<endl;
    EV<< getFullPath() <<" Neighbors gates table: "<<endl;
    map<int, int>::iterator itr3;
    for (itr3 = neighbor_gates.begin(); itr3 != neighbor_gates.end(); ++itr3) {
        EV<< getFullPath() <<" Neighbor: "<< itr3->first <<" Gate: " <<itr3->second <<endl;

    }

}


