///////////////////////////////////////////////////////////////////////////
//
//   MOOS - Mission Oriented Operating Suite 
//  
//   A suit of Applications and Libraries for Mobile Robotics Research 
//   Copyright (C) 2001-2005 Massachusetts Institute of Technology and 
//   Oxford University. 
//    
//   This software was written by Paul Newman and others
//   at MIT 2001-2002 and Oxford University 2003-2005.
//   email: pnewman@robots.ox.ac.uk. 
//      
//   This file is part of a  MOOS Instrument. 
//        
//   This program is free software; you can redistribute it and/or 
//   modify it under the terms of the GNU General Public License as 
//   published by the Free Software Foundation; either version 2 of the 
//   License, or (at your option) any later version. 
//          
//   This program is distributed in the hope that it will be useful, 
//   but WITHOUT ANY WARRANTY; without even the implied warranty of 
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
//   General Public License for more details. 
//            
//   You should have received a copy of the GNU General Public License 
//   along with this program; if not, write to the Free Software 
//   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 
//   02111-1307, USA. 
//
//////////////////////////    END_GPL    //////////////////////////////////



#ifdef _WIN32
#pragma warning(disable : 4786)
#endif


#include <iostream>
#include <math.h>
#include <stdint.h>
#include <string>
#include <map>
#include <algorithm>
#include "MOOS/libMOOS/Comms/MOOSAsyncCommClient.h"
#include "MOOS/libMOOS/Utils/ConsoleColours.h"

extern "C" {
#include "mex.h"
}



//this should deal with the new matlab API
//in versions equal or younger than 7.3.
#ifndef MX_API_VER
 #define DIM_TYPE int
#else
 #if MX_API_VER>=0x07030000
  #define DIM_TYPE mwSize
 #else
  #define DIM_TYPE int
 #endif
#endif




bool  Matlab2Double(double & dfVal,const mxArray * pMLA)
{
    if(!mxIsDouble(pMLA))
        return false;

    dfVal = mxGetScalar(pMLA);
    return true;
}

bool  Matlab2String(std::string & sStr,const mxArray * pMLA)
{
    /* Input must be a string. */
    if ( mxIsChar(pMLA) != 1)
    {
        return false;
    }

    /* Input must be a row vector. */
    if (mxGetM(pMLA)!=1)
    {
        mexPrintf("Input must be a row vector.");
        return false;
    }

    /* Get the length of the input string. */
    int buflen = (mxGetM(pMLA) * mxGetN(pMLA)) + 1;

    /* Allocate memory for input and output strings. */
    void * input_buf=mxCalloc(buflen, sizeof(char));

    /* Copy the string data from prhs[0] into a C string
    * input_ buf.
    * If the string array contains several rows, they are copied,
    * one column at a time, into one long string array.
    */
    int status = mxGetString(pMLA, (char*)input_buf, buflen);

    if(status!=0)
    {
        mexErrMsgTxt("Bad String extraction.");
        return false;
    }

    //yay!
    sStr  = std::string ((char*)input_buf);

    return true;


}

bool Matlab2Binary(std::vector<uint8_t> & bVal, const mxArray * pMLA) {
  if (!mxIsUint8(pMLA) || mxIsEmpty(pMLA)) {
    return false;
  }
  // Input can be either a row or a column vector, but must be a vector.
  if (mxGetM(pMLA) != 1 && mxGetN(pMLA) != 1) {
    mexPrintf("Binary data must be a uint8 vector (row or column).");
    return false;
  }

  const uint8_t* raw_data_start =
      static_cast<const uint8_t*>(mxGetData(pMLA));
  if (raw_data_start == NULL) {
    mexErrMsgTxt("Failed to get pointer to binary data.");
  }

  bVal = std::vector<uint8_t>(
      raw_data_start,
      raw_data_start +mxGetNumberOfElements(pMLA));

  return true;
}






//good to go?
bool bInitialised= false;

//MOOS connection info
std::string sServerHost,sServerPort,sMOOSName;
int lServerPort;

/** a MOOS Connection **/
static CMOOSCommClient* pComms=NULL;

typedef  std::pair<std::string, double> REGINFO;
typedef std::set< REGINFO > REGISTRATION_SET;
static REGISTRATION_SET Registrations;

// a parameter holding class
class Param
{
public:
    enum Type
    {
        STR,
            DBL,
        BIN,
            UNK,
    };
    double dfVal;
    std::string sVal;

    Type m_eType;
    Param()
    {
        m_eType=UNK;
        dfVal = -1;
    }
    std::string Str()
    {
        switch(m_eType)
        {
        case STR:
            return "Str: "+sVal;
        case DBL:
            return MOOSFormat("Dbl: %.3f",dfVal);
          case BIN:
            return "Binary";
        case UNK:
            return MOOSFormat("NOT SET! ");
        }
        return "ERROR";
    }
  bool operator==(double dfV){return dfVal==dfV && m_eType==DBL;};
  bool operator==(std::string sV) {return sVal==sV && m_eType==STR;};
};
typedef  std::map<std::string,Param> ARGMAP;

ARGMAP gArgMap;

void SetParam(std::string sParam,double dfVal)
{
    Param NP;
    NP.m_eType=Param::DBL;
    NP.dfVal = dfVal;
    gArgMap[sParam] = NP;
}

void SetParam(std::string sParam,std::string sVal)
{
    Param NP;
    NP.m_eType=Param::STR;
    NP.sVal = sVal;
    gArgMap[sParam] = NP;
}

void FillDefaultArgMap()
{
    //here we add our default options
    SetParam("MOOSNAME","mexmoos");
    SetParam("SERVERPORT","9000");
    SetParam("SERVERHOST","localhost");

}

bool GetParam(std::string sName, Param & P)
{
    ARGMAP::iterator p = gArgMap.find(sName);
    if(p==gArgMap.end())
    {
        return false;
    }
    else
    {
        P = p->second;
        return true;
    }
}

bool GetDoubleParam(const std::string & sName, double & dfVal)
{
    Param P;
    if( !GetParam(sName, P))
        return false;
    if(P.m_eType!=Param::DBL)
        return false;
    dfVal = P.dfVal;
    return true;
}

bool GetStringParam(const std::string & sName, std::string & sVal)
{
    Param P;
    if( !GetParam(sName, P))
        return false;
    if(P.m_eType!=Param::STR)
        return false;
    sVal = P.sVal;
    return true;
}


std::string GetParamAsString(const std::string & sName)
{
    Param P;
    if( GetParam(sName,P))
    {
        std::stringstream ss;
        switch(P.m_eType)
        {
            case Param::DBL : ss<<P.dfVal; break;
            case Param::STR : ss<<P.sVal; break;
            default:
                break;
        }
        return ss.str();
    }
    return "";
}
Param GetParam(std::string sName)
{
    Param NP;
    ARGMAP::iterator p = gArgMap.find(sName);
    if(p!=gArgMap.end())
    {
        NP = p->second;
    }
    else
    {

        mexPrintf("Warning no such parameter %s %s\n",sName.c_str(),MOOSHERE);
    }
    
    return NP;
}




void OnExit()
{
    
    if(pComms)
    {
        mexPrintf("closing MOOS Comms... ");

        pComms->Close(true);
        delete pComms;
        pComms=NULL;

        mexPrintf("done \n");

    }
    bInitialised = false;

}




void DoRegistrations()
{
    if(pComms!=NULL && pComms->IsConnected())
    {
        REGISTRATION_SET::iterator p;
        for(p = Registrations.begin();p!=Registrations.end();p++)
        {
            pComms->Register(p->first,p->second);
        }
    }
}

bool OnMOOSConnect(void * pParam)
{
    mexPrintf("DB connection established\n");
    DoRegistrations();
    return true;
}


//called the fist time mex-moos runs or when 'init' is passed as teh first parameter
bool Initialise(int nlhs, mxArray *plhs[], const mxArray *prhs[], int nrhs)
{
    
    if(bInitialised)
    {
        mexPrintf("Already initialised - use \"clear mexmoos\" to restart\n");
        return true;
    }
    mexPrintf("* mexmoos initialising *\n");
    
    //get our default args
    FillDefaultArgMap();
    
    //get custom args
    if(prhs!=NULL)
    {
        for(int i = 1;i<nrhs;i+=2)
        {
            if(i<nrhs-1)
            {
                std::string sParam;
                if(!Matlab2String(sParam,prhs[i]))
                {
                    mexErrMsgTxt("Incorrect param value pair (not a string)");
                }
                //MOOSTrace("Read String %s\n",sParam.c_str());
                
                const mxArray *p = prhs[i+1];
                
                Param NewParam;
                NewParam.m_eType = Param::UNK;
                switch(mxGetClassID(p))
                {
                case mxCHAR_CLASS:
                    {
                        std::string sVal;
                        if(Matlab2String(sVal,p))
                        {
                            NewParam.m_eType=Param::STR;
                            NewParam.sVal = sVal;
                        }
                    }
                    break;
                case mxDOUBLE_CLASS:
                    {
                        
                        
                        int nRows = mxGetM(p);
                        int nCols = mxGetN(p);
                        //a scalar
                        NewParam.m_eType = Param::DBL;
                        NewParam.dfVal = mxGetScalar(p);
                        
                    }
                    break;
                default:
                    mexPrintf("Failed to create a parameter\n");
                    break;
                }
                
                //did we parse it OK?
                if(NewParam.m_eType==Param::UNK)
                {
                    mexPrintf("PROBLEM : can't parse parameter value %s\n",sParam.c_str());
                }
                else
                {
                    //store everything in upper case...
                    MOOSToUpper(sParam);
                    ARGMAP::iterator p = gArgMap.find(sParam);
                    
                    if(p==gArgMap.end())
                    {                        
                        mexPrintf("warning no such parameter exists:  %s\n",sParam.c_str());
                    }
                    else
                    {
                        (p->second) = NewParam;
                    }
                }
            }
        }
    }
    
    
    ARGMAP::iterator p;
    
    for(p = gArgMap.begin();p!=gArgMap.end();p++)
    {
        mexPrintf("Property %-25s  %s\n",p->first.c_str(),(p->second).Str().c_str());
    }

    
    //waht to do when we exit?
    mexAtExit(OnExit);
    
    

    // tes 2012-06-20 - write iMatlab configuration to return value [config] = iMatlab('init', ... );
    if(nlhs == 1)
    {
        // transform STRING_LIST into map of key onto values
        std::map<std::string, std::vector<std::string> > params;

        ARGMAP::iterator p;
        for(p = gArgMap.begin();p!=gArgMap.end();p++)
        {
            params[p->first].push_back(GetParamAsString(p->first));

        }

        const int NUMBER_OF_STRUCTS = 1;
        const int NUMBER_OF_FIELDS = params.size();
        mwSize dims[2] = {1, NUMBER_OF_STRUCTS };

        std::vector<const char *> field_names;
        for(std::map<std::string, std::vector<std::string> >::const_iterator it =
                params.begin(), end = params.end(); it != end; ++it)
        {
            field_names.push_back(it->first.c_str());
        }        
        plhs[0] = mxCreateStructArray(2, dims, NUMBER_OF_FIELDS, &field_names[0]);
        
        for(std::map<std::string, std::vector<std::string> >::const_iterator it =
                params.begin(), end = params.end(); it != end; ++it)
        {
            int field = mxGetFieldNumber(plhs[0],it->first.c_str());
            
            std::vector<const char *> values;
            for(std::vector<std::string>::const_iterator jt = it->second.begin(),
                    endj = it->second.end(); jt != endj; ++jt)
            {
                values.push_back(jt->c_str());
            }
            mxSetFieldByNumber(plhs[0],0,field,mxCreateCharMatrixFromStrings(values.size(), &values[0]));
        }        
        
    }


    if(!GetStringParam("MOOSNAME",sMOOSName))
    {
        mexErrMsgTxt("major error cannot retrieve MOOSName");
        return false;
    }

    if(!GetStringParam("SERVERHOST",sServerHost))
    {
        mexErrMsgTxt("major error cannot retrieve SERVERHOST");
        return false;
    }

    if(!GetStringParam("SERVERPORT",sServerPort))
    {
        mexErrMsgTxt("major error cannot retrieve SERVERPORT");
        return false;
    }
    else
    {
        if(!MOOSIsNumeric(sServerPort))
        {
            mexErrMsgTxt("major SERVERPORT is not a number");
            return false;
        }
        lServerPort = atoi(sServerPort.c_str());
    }


    //here we launch the comms
    if(pComms==NULL)
    {
        //pComms = new CMOOSCommClient;
        pComms = new MOOS::MOOSAsyncCommClient;
        MOOS::ConsoleColours::Enable(false);

        pComms->SetOnConnectCallBack(OnMOOSConnect,NULL);
        pComms->Run(sServerHost.c_str(),lServerPort,sMOOSName.c_str());
    }
            


    bInitialised = true;
    return bInitialised;
}


void PrintHelp()
{

    mexPrintf(
            "\n\n** mexmoos:  an interface for MOOS communications inside matlab **\n\n"
            "usage:\n\n"
            "a) Initialisation\n"
            "-----------------------------\n\n"
            "     mexmoos('init')\n\n"
            "   or\n\n"
            "     mexmoos('init',config_param_name,config_param_val,....)\n\n"
            "   Where <config_param_name,config_param_val> are parameter value pairs.\n"
            "   All initialisation parameters are strings. The following are supported. \n\n"
            "     SERVERHOST name of machine hosting DB (default localhost)\n"
            "     SERVERPORT port (as string e.g '9001') on which DB is serving (default 9000)\n"
            "     MOOSNAME   name with which to register (default mexmoos)\n"
            "\n\n"
            "   example:\n"
            "     mexmoos('init','SERVERHOST','vader.robots.ox.ac.uk','MOOSNAME','the-darkness')\n\n"
            ""
            "b) Registration\n"
            "-----------------------------\n\n"
            "    mexmoos('REGISTER',varname,period)\n\n"
            "  examples\n\n"
            "    mexmoos('REGISTER','laser2d',0.0)\n"
            "    mexmoos('REGISTER','laser2d',0.1)\n\n"
            "  The legacy command MOOS_REGISTER can be substituted for  'REGISTER'\n\n"
            "c) Notification (Sending)\n"
            "-----------------------------\n\n"
            "    mexmoos('NOTIFY',varname,data)\n\n"
            "  examples\n\n"
            "    mexmoos('NOTIFY','varA',pi)\n"
            "    mexmoos('NOTIFY','varA','hello world')\n"
            "    mexmoos('NOTIFY','varA',zeros(1,3,'uint8'))\n\n"
            "  The legacy command 'MOOS_MAIL_TX' can be substituted for 'NOTIFY'\n\n"
            ""
            "d) Receiving (Reading)\n"
            "-----------------------------\n\n"
            "    mexmoos('FETCH')\n\n"
            "  example\n\n"
            "    msgs=mexmoos('FETCH')\n\n"
            "\n"
            "  Returns an array of msg structures - each containing moos message data.\n"
            "  The legacy command 'MOOS_MAIL_RX' can be substituted for  'FETCH'\n\n"

            ""
            "e) Closing\n"
            "-----------------------------\n\n"
            "    mexmoos('CLOSE')\n\n"
            "e) Help\n"
            "-----------------------------\n\n"
            "    mexmoos('HELP')\n\n");


    
}



//--------------------------------------------------------------
// function: mex_moos - Entry point from Matlab environment (via
//   mexFucntion(), below)
// INPUTS:
//   nlhs - number of left hand side arguments (outputs)
//   plhs[] - pointer to table where created matrix pointers are
//            to be placed
//   nrhs - number of right hand side arguments (inputs)
//   prhs[] - pointer to table of input matrices
//--------------------------------------------------------------
void mex_moos( int nlhs, mxArray *plhs[], int nrhs, const mxArray  *prhs[] )
{
    // TODO: Add your application code here
    if(nrhs==0)
    {
        //no argument - print help
        PrintHelp();
        return;
    }
    
    
    std::string sCmd;
    //first parameter is always a string  command
    if(!Matlab2String(sCmd,prhs[0]))
    {
        mexErrMsgTxt("Param 1 (cmd) must be a string ");
    }
    
    
    if(MOOSStrCmp(sCmd,"INIT"))
    {
        Initialise(nlhs, plhs, prhs,nrhs);
        return;
    }
    else if(MOOSStrCmp(sCmd,"HELP"))
    {
        PrintHelp();
        return;
    }

    if(!bInitialised)
    {
        MOOSTrace("iMatlab is not initialised - must call \"iMatlab('init')\" first\n");
        return;
    }



    /// SENDING MOOS MAIL
    if(MOOSStrCmp(sCmd,"MOOS_MAIL_TX") || MOOSStrCmp(sCmd,"NOTIFY") )
    {
        if(nrhs<3)
        {
            MOOSTrace("Incorrect format : 'MOOS_MAIL_TX','VAR_NAME'.VarVal (string, double or uint8)\n");
            return ;
        }
        std::string sKey;
        if(!Matlab2String(sKey,prhs[1]))
        {
            mexErrMsgTxt("Param 2 (key) must be a string name of the data being sent\n");
        }

        if(pComms && pComms->IsConnected())
        {
            double dfTime=MOOSTime();
            if(nrhs==4 && ! Matlab2Double(dfTime,prhs[3]))
            {
                mexErrMsgTxt("parameter 4 must be a  double time\n");
            }

            std::string sTmp;
            double dfTmp;
            std::vector<uint8_t> bTmp;
            if(Matlab2String(sTmp,prhs[2]))
            {
                pComms->Notify(sKey,sTmp,dfTime);
            }
            else if(Matlab2Double(dfTmp,prhs[2]))
            {
                pComms->Notify(sKey,dfTmp,dfTime);
            }
            else if(Matlab2Binary(bTmp,prhs[2]))
            {
                pComms->Notify(sKey, bTmp, dfTime);
            }
            else
            {
                mexErrMsgTxt("MOOS transmit failed parameter 3 must be a string or double data value\n");
            }
        }
        else
        {
            mexErrMsgTxt("MOOS transmit failed - not connected\n");
        }
    }
    //COLLECTING MOOS MAIL FROM COMMS THREAD
    else if(MOOSStrCmp(sCmd,"MOOS_MAIL_RX") ||  MOOSStrCmp(sCmd,"FETCH") )
    {
        if(pComms->IsConnected())
        {
            MOOSMSG_LIST NewMail;
            if(pComms->Fetch(NewMail))
            {
                //how many have we got?
                int nMsgs = NewMail.size();

                if(nlhs==1)
                {


                    //make a struct array

                    DIM_TYPE  DimArray[2];
                    DimArray[0] = 1;DimArray[1] = nMsgs;

                    const char * FieldNames[] = {"KEY","TYPE","TIME","STR","DBL","BIN","SRC","ORIGINATING_COMMUNITY"};

                    plhs[0] = mxCreateStructArray(2, DimArray, sizeof(FieldNames)/sizeof(char*),FieldNames);

                    int nKeyField = mxGetFieldNumber(plhs[0],FieldNames[0]);
                    int nTypeField = mxGetFieldNumber(plhs[0],FieldNames[1]);
                    int nTimeField = mxGetFieldNumber(plhs[0],FieldNames[2]);
                    int nStrField = mxGetFieldNumber(plhs[0],FieldNames[3]);
                    int nDblField = mxGetFieldNumber(plhs[0],FieldNames[4]);
                    int nBinField = mxGetFieldNumber(plhs[0],FieldNames[5]);
                    int nSrcField = mxGetFieldNumber(plhs[0],FieldNames[6]);
                    int nCommunityField = mxGetFieldNumber(plhs[0],FieldNames[7]);


                    MOOSMSG_LIST::iterator p;

                    int i = 0;
                    for(p = NewMail.begin();p!=NewMail.end();p++,i++)
                    {
                        //copy in the Key of the variable
                        mxSetFieldByNumber(plhs[0],i,nKeyField,mxCreateString(p->m_sKey.c_str()));
                        
                        //copy in the type
                        std::string pType("UNKNOWN");
                        if (p->IsDataType(MOOS_DOUBLE)) {
                          pType = "DBL";
                        } else if (p->IsDataType(MOOS_STRING)) {
                          pType = "STR";
                        } else if (p->IsDataType(MOOS_BINARY_STRING)) {
                          pType = "BIN";
                        }
                        //char * pType = (char*)(p->IsDataType(MOOS_DOUBLE) ? "DBL":"STR");
                        mxSetFieldByNumber(plhs[0],i,nTypeField,mxCreateString(pType.c_str()));
                        
                        //copy in time
                        mxSetFieldByNumber(plhs[0],i,nTimeField,mxCreateDoubleScalar(p->GetTime()));
                        
                        //copy in sVal
                        mxSetFieldByNumber(plhs[0],i,nStrField,mxCreateString(p->m_sVal.c_str()));
                        
                        //copy in dfVal
                        mxSetFieldByNumber(plhs[0],i,nDblField,mxCreateDoubleScalar(p->GetDouble()));

                        //copy in bVal
                        const std::vector<uint8_t> binaryData = p->GetBinaryDataAsVector();
                        mxArray* binaryDataMatlab = mxCreateNumericMatrix(1, binaryData.size(),
                                                                          mxUINT8_CLASS, mxREAL);
                        std::copy(binaryData.begin(),
                                  binaryData.end(),
                                  static_cast<uint8_t*>(mxGetData(binaryDataMatlab)));
                        mxSetFieldByNumber(plhs[0],i,nBinField,binaryDataMatlab);

                        //copy in src process
                        mxSetFieldByNumber(plhs[0],i,nSrcField,mxCreateString(p->m_sSrc.c_str()));

                        //copy in originating community
                        mxSetFieldByNumber(plhs[0],i,nCommunityField,mxCreateString(p->m_sOriginatingCommunity.c_str()));

                    }
                }
                else
                {
                    MOOSTrace("Picked up %d MOOSMsgs - but no output variables to return them in!\n",nMsgs);
                }

            }
            else
            {
                //the Fetch failed but we are connected - probably no data
                //make a struct array
                DIM_TYPE DimArray[2];DimArray[0] = 1;DimArray[1] = 0;
                const char * FieldNames[] = {"KEY","TYPE","TIME","STR","DBL","BIN","SRC","ORIGINATING_COMMUNITY"};
                plhs[0] = mxCreateStructArray(2, DimArray, sizeof(FieldNames)/sizeof(char*),FieldNames);
            }
        }
        else
        {
            MOOSTrace("No MOOS connection established Mail Rx failed\n");
            DIM_TYPE DimArray[2];DimArray[0] = 1;DimArray[1] = 0;
            const char * FieldNames[] = {"KEY","TYPE","TIME","STR","DBL","BIN","SRC","ORIGINATING_COMMUNITY"};
            plhs[0] = mxCreateStructArray(2, DimArray, sizeof(FieldNames)/sizeof(char*),FieldNames);


            return;
        }
    }
    //REGISTERING FOR MAIL
    else if(MOOSStrCmp(sCmd,"MOOS_REGISTER") || MOOSStrCmp(sCmd,"REGISTER") )
    {
        if(nrhs!=3)
        {
            MOOSTrace("incorrect format. Use iMatlab('MOOS_REGISTER','WHAT',HOW_OFTEN\n");
            return;
        }
        else
        {

            std::string sWhat;

            if(!Matlab2String(sWhat,prhs[1]))
            {
                MOOSTrace("incorrect format parameter 2 must be a string\n");
                return;
            }
            double dfHowOften;
            if(!Matlab2Double(dfHowOften,prhs[2]))
            {
                MOOSTrace("incorrect format parameter 3 must be a double (min period between messages)\n");
                return;
            }

            //save the information
            Registrations.insert( REGINFO(sWhat,dfHowOften) );

            //reregister
            DoRegistrations();
        }
    }
    //PAUSING
    else if(MOOSStrCmp(sCmd,"MOOS_PAUSE"))
    {
        double dfT;
        if(!Matlab2Double(dfT,prhs[1]))
        {
            MOOSFail("incorrect MOOS_PAUSE format - param 2 must be  numeric seconds\n");
            return;
        }
        MOOSPause(static_cast<int> (dfT*1000.0));

    }
    //REGISTERING FOR MAIL
    else if(MOOSStrCmp(sCmd,"CLOSE"))
    {
        OnExit();
        return;
    }
    else
    {
        MOOSTrace("Huh? - command %s is not known\n",sCmd.c_str());
    }

    
    
    
} // end mex_moos()



extern "C" {
    //--------------------------------------------------------------
    // mexFunction - Entry point from Matlab. From this C function,
    //   simply call the C++ application function, above.
    //--------------------------------------------------------------
    void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray  *prhs[] )
    {
        mex_moos(nlhs, plhs, nrhs, prhs);
    }
}


