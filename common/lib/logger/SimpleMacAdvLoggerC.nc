
/*
 * Copyright (c) 2007 University of Copenhagen
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of University of Copenhagen nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY
 * OF COPENHAGEN OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Log recording layer
 *
 * @author Marcus Chang <marcus@diku.dk>
 * @author Martin Leopold <leopold@diku.dk>
 */

configuration SimpleMacAdvLoggerC {
    provides interface StdControl;
    provides interface SimpleMac;
}

implementation {
    components MainC, SimpleMacM;
    
    MainC.SoftwareInit -> SimpleMacM;

    StdControl = SimpleMacM;
    SimpleMac = SimpleMacM;
    
    SimpleMacM.HALCC2420Control -> HALCC2420C.StdControl;
    SimpleMacM.HALCC2420 -> HALCC2420C.HALCC2420;

#ifdef __micro4__
    components HALCC2420LoggerC as HALCC2420C;
#elif __cc2430em__
    components HalCC2430LoggerC as HALCC2420C;
#endif

}

