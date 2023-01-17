import { error, json, type RequestHandler } from '@sveltejs/kit';
import status from 'http-status';

export const POST: RequestHandler = async ({ params, request, platform }) => {
	const messageFromEnigma = params.enigmaMessage;//haalt message uit url

    //checkt of er een api key is
    if (!request.headers.has('x-api-key')) throw error(status.BAD_REQUEST, 'Missing X-API-Key'); 
    //checkt of api key juist is 
    if (request.headers.get('x-api-key') !== platform.env.API_TOKEN)
		throw error(status.UNAUTHORIZED, 'Invalid API key');

    await platform.env.MESSAGE.put("enigmaMessage", messageFromEnigma);

    return json(
        { status: status.CREATED }
    );
}
