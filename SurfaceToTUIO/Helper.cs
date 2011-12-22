using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections.Specialized;
using System.Net;

namespace SurfaceToTUIO
{
    public class Helper
    {
        public static string getLocalIP()
        {

            // originally, this code was using Dns.GetHostEntry
            // to find ip address of this machine,
            // which would return different quantities and ipaddresses
            // depending on if the Surface was connected to the internet or not.

            // We only want the Surface Input to be transmitted locally,
            // so I've simplified this by always returning a local address.
            return "127.0.0.1";
        }
    }
}
