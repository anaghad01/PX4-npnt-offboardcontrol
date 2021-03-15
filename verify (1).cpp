//g++ verify.cpp libchilkat-9.5.0.a -lpthread -lresolv

#include "chilkat-9.5.0/include/CkStringBuilder.h"
#include "chilkat-9.5.0/include/CkXmlDSig.h"
#include "iostream"
using namespace std;

void ChilkatSample(void)
{
    CkStringBuilder sbXml;
    //Loads the XML file (Add path of file and charset)
    bool success = sbXml.LoadFile("permission_artifact_1.xml","utf-8");
    if (success != true) {
        cout << "Failed to load XML file." << "\r\n";
        return;
    }

    CkXmlDSig dsig;

    //Loads the XML containing the signatures to be verified.
    success = dsig.LoadSignatureSb(sbXml);
    if (success != true) {
        cout << dsig.lastErrorText() << "\r\n";
        return;
    }

    int i = 0;
    while (i < dsig.get_NumSignatures()) {
        // Select the Nth signature by setting the Selector property.
        dsig.put_Selector(i);

        // The bVerifyReferenceDigests argument determines if we want
        // to also verify each reference digest.  If set to false,
        // then only the SignedInfo part of the Signature is verified.
        bool bVerifyReferenceDigests = true;
        bool bVerified = dsig.VerifySignature(bVerifyReferenceDigests);
        cout << "Signature " << (i + 1) << " verified = " << bVerified << "\r\n";

        i = i + 1;
    }
    }

int main()
{
	ChilkatSample();
}
