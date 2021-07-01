using System;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Azure.WebJobs;
using Microsoft.Azure.WebJobs.Extensions.Http;

namespace Glovebox.Function
{
    public static class GetMqttCredentials
    {
        [FunctionName("MQTT_Credentials")]
        public static IActionResult Run([HttpTrigger(AuthorizationLevel.Anonymous, "get", "post", Route = "mqtt-settings")] HttpRequest req)
        {
            return new OkObjectResult(new
            {
                userName = Environment.GetEnvironmentVariable("mqtt_username"),
                password = Environment.GetEnvironmentVariable("mqtt_password"),
                broker = Environment.GetEnvironmentVariable("mqtt_broker"),
                brokerPort = Environment.GetEnvironmentVariable("mqtt_broker_port")
            });
        }
    }
}
