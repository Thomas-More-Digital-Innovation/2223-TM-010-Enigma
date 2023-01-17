import { error } from '@sveltejs/kit';
import type { PageServerLoad } from './$types';
 
export const load = (async ({ params, platform }) => {
    const messageFromEnigma = await platform?.env.MESSAGE.get("enigmaMessage");
    return {
        messageFromEnigma
    }
}) satisfies PageServerLoad;