import { createClient } from '@supabase/supabase-js';

// For Docusaurus, environment variables are available in process.env
// Use the standard names in .env.local file
const supabaseUrl = process.env.SUPABASE_URL || process.env.REACT_APP_SUPABASE_URL;
const supabaseAnonKey = process.env.SUPABASE_ANON_KEY || process.env.REACT_APP_SUPABASE_ANON_KEY;

if (!supabaseUrl || !supabaseAnonKey) {
  console.warn('Missing Supabase environment variables. Using fallback values.');
  console.warn('Expected SUPABASE_URL and SUPABASE_ANON_KEY environment variables.');
}

const supabase = createClient(
  supabaseUrl || 'https://borcjbmeyzdrfnkyjnnp.supabase.co', // fallback for development
  supabaseAnonKey || 'sb_publishable_TTiEFXDNbsDx2_4s0aq1Ew_2QOy6qDK' // fallback for development
);

export default supabase;