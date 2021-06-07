// using System;
// using System.Net.Http;
// using System.Text;
// using System.Threading.Tasks;
// using Microsoft.AspNetCore.Http;
// using Microsoft.AspNetCore.Mvc;
// using Microsoft.Azure.WebJobs;
// using Microsoft.Azure.WebJobs.Extensions.Http;
// using Microsoft.Extensions.Logging;


// // https://docs.microsoft.com/en-us/rest/api/iotcentral/devices/getproperties


// namespace Glovebox.Function
// {
//     public static class SetChannelId
//     {
//         private static string authorization = Environment.GetEnvironmentVariable("IOT_CENTRAL_API_TOKEN");
//         // private static string deviceId = Environment.GetEnvironmentVariable("IOT_CENTRAL_DEVICE_ID");
//         private static string iotCentralUrl = Environment.GetEnvironmentVariable("IOT_CENTRAL_URL");

//         [FunctionName("control")]
//         public static async Task<IActionResult> Run(
//             [HttpTrigger(AuthorizationLevel.Anonymous, "get", "post", Route = null)] HttpRequest req,
//             ILogger log)
//         {
//             StringBuilder sb = new StringBuilder();

//             string channelId = req.Query["channelid"];
//             string deviceId = req.Query["deviceid"];

//             if (string.IsNullOrEmpty(channelId) || string.IsNullOrEmpty(deviceId))
//             {
//                 return new BadRequestObjectResult("failed, missing channel Id or device id");
//             }

//             int channel_id = 0;
//             if (!int.TryParse(channelId, out channel_id))
//             {
//                 return new BadRequestObjectResult("failed, channel id not numeric");
//             }

//             if (channel_id <= 0 ){
//                 return new BadRequestObjectResult("failed, channel id must be great than zero");
//             }

//             sb.Append("{\"DesiredChannelId\":");
//             sb.Append(channelId);
//             sb.Append("}");

//             using (var client = new HttpClient())
//             {
//                 client.BaseAddress = new Uri(iotCentralUrl);
//                 client.DefaultRequestHeaders.Add("Authorization", authorization);

//                 var content = new StringContent(sb.ToString(), Encoding.UTF8, "application/json");

//                 var api = $"api/preview/devices/{deviceId.ToLower()}/properties";

//                 var response = await client.PutAsync(api, content);
//                 if (response.IsSuccessStatusCode)
//                 {
//                     return new OkObjectResult("success");
//                 }
//                 else
//                 {
//                     return new OkObjectResult("failed");
//                 }
//             }
//         }
//     }
// }
