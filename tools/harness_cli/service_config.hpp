// Public Supabase client configuration shared with the browser bundle.
// This is the anonymous/publishable key, never the service-role key. Database
// row-level security and the narrow RPC functions enforce authorization.
#ifndef HARNESS_CLI_SERVICE_CONFIG_HPP
#define HARNESS_CLI_SERVICE_CONFIG_HPP

namespace harness::cli {
inline constexpr char kDefaultSupabaseUrl[] = "https://dqyjpizwaldtoqpbrarp.supabase.co";
inline constexpr char kDefaultSupabaseAnonKey[] = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImRxeWpwaXp3YWxkdG9xcGJyYXJwIiwicm9sZSI6ImFub24iLCJpYXQiOjE3ODk3NTIzNTksImV4cCI6MjEwNTMyODM1OX0.hT_M6wlfcioCd6T-12aoczN4MF1lphGdUTL19QFZrME";
}

#endif
