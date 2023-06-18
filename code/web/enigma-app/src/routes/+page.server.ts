import type { PageServerLoad } from './$types';
 
export const load = (async ({ platform }) => {
    if (!platform) {
        return {
            messageFromEnigma: "Local test"
        }
    }

    const messageFromEnigma = await platform.env.MESSAGE.get("enigmaMessage");
    return {
        messageFromEnigma
    }
}) satisfies PageServerLoad;