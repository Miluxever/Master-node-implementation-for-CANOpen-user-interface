#include "Master.h"
#include <cstdint>


/*Implementazione di un'interfaccia software per il protocollo CANOpen*/
bool_t NMTisPreOrOperational;

/*void Master::start_init_callbacks(uint8_t number_of_nodes, void (*Function_to_call)){

    for(int i=0; i++; i < number_of_nodes){

        CO_HBconsumer_initCallbackRemoteReset(CO->HBcons, i, NULL, (Function_to_call)(uint8_t, uint8_t, void *));

    }

}*/


Master::Master(/*CO_OD_entry_t* od,*/ uint16_t bitrate, uint8_t node_id){//, PinName can_tx, PinName can_rx){

    this->Bitrate = bitrate;
    this->Node_Id = node_id;
    //this->Can_Tx = can_tx;
    //this->Can_Rx = can_rx;
    //this->Object_Dictonary = od;
    //mbed::CAN can_bus(Can_Rx, Can_Tx, Bitrate);
    CO_ReturnError_t err = CO_init(
            (void *)(0x40000000UL + 0x6400UL) /*(APB1PERIPH_BASE + 0x6400UL)*/,    //CANOPEN_DEFAULT_NODE_ID, CAN_BITRATE
            Node_Id,Bitrate);

    CO_NMT_reset_cmd_t resetCmd = CO_process(CO, 0, NULL); //chiama CO_NMT_process, la quale imposta lo stato in preoperational
    
    NMTisPreOrOperational = true;

}

///////////////////////////////////////////////////////////////////
///////////////////////     NMT FUNCTIONS   ///////////////////////
///////////////////////////////////////////////////////////////////

void Master::NMT_Set_Operational_State(){

    if(CO->NMT->operatingState == CO_NMT_PRE_OPERATIONAL || CO->NMT->operatingState == CO_NMT_STOPPED) CO_NMT_setInternalState(CO->NMT, CO_NMT_OPERATIONAL);

}

void Master::NMT_Start_remote_node(uint8_t slave_id){

    CO_ReturnError_t err = CO_sendNMTcommand(CO, CO_NMT_ENTER_OPERATIONAL, slave_id);

}

void Master::NMT_Enter_pre_operational(uint8_t slave_id){

    CO_ReturnError_t err = CO_sendNMTcommand(CO, CO_NMT_ENTER_PRE_OPERATIONAL, slave_id);

}

void Master::NMT_Reset_node(uint8_t slave_id){

    CO_ReturnError_t err = CO_sendNMTcommand(CO, CO_NMT_RESET_NODE, slave_id);

}   

void Master::NMT_Reset_communication(uint8_t slave_id){

    CO_ReturnError_t err = CO_sendNMTcommand(CO, CO_NMT_RESET_COMMUNICATION, slave_id);

}   

void Master::NMT_Stop_remote_node(uint8_t slave_id){

    CO_ReturnError_t err = CO_sendNMTcommand(CO, CO_NMT_ENTER_STOPPED, slave_id);

}

///////////////////////////////////////////////////////////////////
///////////////////////     SYNC FUNCTIONS   //////////////////////
///////////////////////////////////////////////////////////////////


/*bool Master::SYNC_Start_SYNC_protocol(){

    if(CO->SYNC->isProducer) {
    
        CO_SYNC_process(CO->SYNC, 1000, CO_OD_ROM.synchronousWindowLength);
        return true;

    }

    else return false;

}


bool Master::SYNC_Set_Communication_cycle_period(uint32_t Communication_cycle_period_ms){

    uint8_t* val_p;
    *val_p = Communication_cycle_period_ms;

    CO_SDOclient_return_t r;

    uint8_t id = get_nodeid();

    r = CO_SDOclient_setup(CO->SDOclient[0], 0, 0, 10); 
    
    r = CO_SDOclientDownloadInitiate(CO->SDOclient[0],
                                     0x1006,
                                     0, 
                                     val_p, 
                                     sizeof(*val_p), 
                                     0);

    uint32_t* abort;
    if(CO_SDOclientDownload(CO->SDOclient[0], 1000, 5000, abort) == 0) return true;
    else return false;

}


bool Master::SYNC_Stop_SYNC_protocol(){

    uint8_t* val_p;
    *val_p = 0;

    CO_SDOclient_return_t r;

    r = CO_SDOclient_setup(CO->SDOclient[0], 0, 0, get_nodeid()); 
    
    r = CO_SDOclientDownloadInitiate(CO->SDOclient[0],
                                     0x1006,
                                     0, 
                                     val_p, 
                                     sizeof(*val_p), 
                                     0);

    uint32_t* abort;
    if(CO_SDOclientDownload(CO->SDOclient[0], 1000, 5000, abort) == 0) return true;
    else return false;

}


bool Master::SYNC_Set_counter(uint8_t value){

    if(value <= 1 || value > 240) return false;

    uint8_t* val_p;
    *val_p = value;

    CO_SDOclient_return_t r;

    r = CO_SDOclient_setup(CO->SDOclient[0], 0, 0, get_nodeid()); 
    
    r = CO_SDOclientDownloadInitiate(CO->SDOclient[0],
                                     0x1019,
                                     0, 
                                     val_p, 
                                     sizeof(value), 
                                     0);

    uint32_t* abort;
    if(CO_SDOclientDownload(CO->SDOclient[0], 1000, 5000, abort) == 0) return true;
    else return false;

}
//typedef void (*Function_to_call)(uint8_t,uint8_t,void*);


/*void Master::Set_Bootup_callback(uint8_t nodeId, Function_to_call FN){

    //uint8_t nodeid = CO->HBcons->monitoredNodes[index_of_the_node].nodeId;
    int8_t index = CO_HBconsumer_getIdxByNodeId(CO->HBcons, nodeId);

    if(index<0) return;
    
    CO_HBconsumer_initCallbackRemoteReset(CO->HBcons, index, NULL, FN);

}*/
//(Function_to_call)(nodeid,index_of_the_node,NULL)

///////////////////////////////////////////////////////////////////
///////////////////////     TIME FUNCTIONS   //////////////////////
///////////////////////////////////////////////////////////////////

void Master::TIME_Set_global_network_time(){

    if(CO->TIME->isProducer) CO_TIME_process(CO->TIME, 100);

}

///////////////////////////////////////////////////////////////////
///////////////////////     SDO FUNCTIONS   ///////////////////////
///////////////////////////////////////////////////////////////////

void Master::SDO_Download_SDO(uint8_t slave_id, uint16_t index, uint8_t subIndex, uint8_t *dataTx, uint32_t dataSize){

    CO_SDOclient_return_t r;

    if(CO->SDOclient[0]->SDOClientPar->nodeIDOfTheSDOServer != slave_id){

        r = CO_SDOclient_setup(CO->SDOclient[0], 
                               0, 
                               0, 
                               slave_id);   //ok

    }
    
    r = CO_SDOclientDownloadInitiate(CO->SDOclient[0],
                                     index,
                                     subIndex, 
                                     dataTx, 
                                     dataSize, 
                                     0);  //ok
    
    uint32_t abort;

    if(CO->SDOclient[0]->SDOClientPar->nodeIDOfTheSDOServer == CO->SDO[0]->nodeId) CO_SDOclientDownload(CO->SDOclient[0], 1000, 5000, &abort);

    //else while(CO_SDOclientDownload(CO->SDOclient[0], 1000, 5000, &abort) != 0);
    CO_SDOclientDownload(CO->SDOclient[0], 1000, 5000, &abort);
    CO_SDOclientClose(CO->SDOclient[0]);


}

void Master::SDO_Upload_SDO(uint8_t slave_id, uint16_t index, uint8_t subIndex, uint8_t* dataRx, uint32_t dataSize, uint32_t* data_size, uint32_t* abort){

    CO_SDOclient_return_t r;

    if(CO->SDOclient[0]->SDOClientPar->nodeIDOfTheSDOServer != slave_id){

        r = CO_SDOclient_setup(CO->SDOclient[0], 
                               0, 
                               0, 
                               slave_id);   //ok

    }

    r = CO_SDOclientUploadInitiate(CO->SDOclient[0],
                                   index,
                                   subIndex, 
                                   dataRx, 
                                   dataSize, 
                                   0);  //ok

    
    /*uint32_t datasize;
    uint32_t abort;*/

    CO_SDOclientUpload(CO->SDOclient[0], 1000, 10000, data_size, abort);
    /*if(CO->SDOclient[0]->SDOClientPar->nodeIDOfTheSDOServer == CO->SDO[0]->nodeId) CO_SDOclientUpload(CO->SDOclient[0], 1000, 5000, data_size, abort);

    else while(CO_SDOclientUpload(CO->SDOclient[0], 1000, 5000, data_size, abort) != 0);*/

    //CO_SDOclientClose(CO->SDOclient[0]);
}

///////////////////////////////////////////////////////////////////
///////////////////////     HB FUNCTIONS   ////////////////////////
///////////////////////////////////////////////////////////////////

void Master::HB_Start_produce_HB(){

    CO_NMT_reset_cmd_t start = CO_NMT_process(CO->NMT, 
                                              1, 
                                              OD_producerHeartbeatTime, 
                                              OD_NMTStartup, 
                                              OD_errorRegister, 
                                              OD_errorBehavior, 
                                              NULL);
    
}

void Master::HB_Set_slave_hb_producer_time(uint8_t slave_id, uint8_t index, uint16_t hb_producer_time_ms, uint16_t hb_consumer_time_ms){

    if(index < 0 || index >= CO->HBcons->numberOfMonitoredNodes /*|| hb_producer_time_ms == 0*/) return;

    uint8_t bufferTx[2];
    CO_setUint16(bufferTx, hb_producer_time_ms);

    Master::SDO_Download_SDO(1,
                             0x1017,
                             0,
                             bufferTx,
                             2);

    CO_HBconsumer_initEntry(CO->HBcons, index, slave_id, hb_consumer_time_ms);

}

void Master::HB_Start_process_HB(){

    CO_HBconsumer_process(CO->HBcons, 
                          NMTisPreOrOperational,
                          1);

}

bool Master::HB_Set_producer_hb_time(uint16_t time_ms){

    uint8_t bufferTx[2];

    CO_setUint16(bufferTx, time_ms);

    CO_SDOclient_return_t r;

    r = CO_SDOclient_setup(CO->SDOclient[0], 0, 0, get_nodeid()); 
    
    r = CO_SDOclientDownloadInitiate(CO->SDOclient[0],
                                     0x1017,
                                     0, 
                                     bufferTx, 
                                     2, 
                                     0);

    uint32_t abort;
    if(CO_SDOclientDownload(CO->SDOclient[0], 1, 10000, &abort) == 0) return true;

    else return false;

}

/*void Master::HB_Set_hb_callback(uint8_t node_id, void (*Function_to_call)){

    int8_t index = CO_HBconsumer_getIdxByNodeId(CO->HBcons, node_id);

    CO_HBconsumer_initCallbackHeartbeatStarted(CO->HBcons, index, NULL, (Function_to_call)(node_id, index, NULL));

}*/