(* Copyright (C) 2017,2019 Jason Carr, Matthew Fluet.
 *
 * MLton is released under a HPND-style license.
 * See the file MLton-LICENSE for details.
 *)

structure Parse :> PARSE =
struct

infix 1 <|> >>=
infix  3 <*> <* *>
infixr 4 <$> <$$> <$$$> <$$$$> <$ <$?> 

structure Location =
   struct
      type t = {line: int, column: int}
   end
structure State =
   struct
      (* The state primarily consists of the stream of future
       * characters. We keep a buffer of the last string read
       * annotated with the appropriate locations. Hopefully, 
       * it should be created only when needed, if the functions
       * are sufficiently inlined *)
      type t = T of {
         buffer: string,
         (* Invariant: must be same length as buffer *)
         locations: Location.t vector,
         position: int,
         stream: StreamIO.instream}

      val lastLocation (T {locations, ...}) =
         if Vector.isEmpty locations
         then NONE
         else SOME (Vector.last locations)
   end

datatype ('a, 'b) result = Success of 'a * 'b
                         | Expected of string list

type 'a t = T of
   {mayBeEmpty: bool,
    firstChars: char list option,
    run: (State.t -> (State.t, 'a) result)}

fun indexLocations ({line, column}, s) =
   Vector.tabulate (String.length s,
      fn i =>
         if String.sub (s, i) = #"\n"
         then {line=line+1, column=0}
         else {line=line, column=column+1})

fun getNext (State.T {buffer, locations, position, stream}):
      (char * Location.t * State.T) option =
   let
      fun fillNext () =
         let
            val lastLocation = Vector.last locations
            val (buffer, stream) = StreamIO.input stream
            val locations = indexLocations (buffer, lastLocation)
         in
            State.T {buffer=buffer, locations=locations,
                     position=0, stream=stream}
         end
   in
      case Int.compare (position, String.length buffer - 1) of
           LESS =>
               SOME (String.sub (buffer, i),
                     Vector.sub (locations, i),
                State.T {buffer=buffer, locations=locations,
                         position=position+1, stream=stream})
         | EQ =>
               SOME (String.sub (buffer, i), Vector.sub (locations, i), fillNext ())
         | GREATER =>
              let
                 state as State.T {buffer, locations, position, stream} = fillNext ()
              in
                 if String.length buffer = 0
                 then NONE
                 else SOME
                  (Vector.first buffer,
                   Vector.first locations,
                   State.T {buffer=buffer, locations=locations,
                            position=position+1, stream=stream})
   end


fun doFail []  = Result.No ("Parse error")
  | doFail([msg]) = Result.No ("Parse error: Expected " ^ msg)
  | doFail(msgs) = Result.No ("Parse error: Expected one of \n" ^
       (String.concat(List.map(msgs, fn x => x ^ "\n"))))

fun parseStream(p : 'a t, stream) : 'a Result.t =
   case p (indexStream({line=1, column=1}, stream))
   of Success (b, _) => Result.Yes b
    | Failure ms => doFail ms
    | FailCut ms => doFail ms
fun parseString(p : 'a t, string) : 'a Result.t =
   parseStream(p, Stream.fromList (String.explode string))
fun parseFile(p : 'a t, file) : 'a Result.t =
   File.withIn
   (file, fn i =>
    let
       fun toStream () =
          case In.inputChar i of
             SOME c => Stream.cons (c, Stream.delay toStream)
           | NONE => Stream.empty ()
    in
       parseStream(p, toStream ())
    end)


fun unionFirstChars (c1, c2) =
   case (c1, c2) of
        (SOME cs1, SOME cs2) => SOME (List.union (cs1, cs2, op =))
       _ => NONE

fun (T {mayBeEmpty=empty1, firstChars=chars1, run=run1})
     <*>
    (T {mayBeEmpty=empty2, firstChars=chars2, run=run2}) =
   T {mayBeEmpty=empty1 and empty2,
      firstChars=
         if empty1
         then unionFirstChars (chars1, chars2)
         else firstChars1,
      run=fn s =>
          case run1 s of
              Success (f, s') =>
                  (case run2 s' of
                       Success (b, s'') =>
                        Success (f b, s'')
                     | Failure err => Failure err)
            | Failure err => Failure err)}

fun (T {mayBeEmpty, firstChars, run}) >>= f =
   T {mayBeEmpty=mayBeEmpty,
      firstChars=
         if mayBeEmpty
         then NONE
         else firstChars,
      run=fn s =>
         case run s of
              Success (a, s') =>
                 #run f a s'
            | Failure err => Failure err}

fun fst a _ = a
fun snd _ b = b

fun curry f a b = f (a, b)
fun curry3 f a b c = f (a, b, c)
fun curry4 f a b c d = f (a, b, c, d)

fun pure a =
   T {mayBeEmpty=true,
      firstChars=NONE,
      run=fn s => (a, s)}

fun f <$> p = (pure f) <*> p
fun f <$$> (p1, p2) = curry <$> (pure f) <*> p1 <*> p2
fun f <$$$> (p1, p2, p3) = curry3 <$> (pure f) <*> p1 <*> p2 <*> p3
fun f <$$$$> (p1, p2, p3, p4) = curry4 <$> (pure f) <*> p1 <*> p2 <*> p3 <*> p4
fun f <$?> p = p >>= (fn a => case f a of SOME b => pure b
                                        | NONE => fn _ => Failure [])
fun a <* b = fst <$> a <*> b
fun a *> b = snd <$> a <*> b
fun v <$ p = (fn _ => v) <$> p
fun a <|> b = fn s => case (a s)
   of Success r => Success r
    | Failure err1 => (case (b s) of
        Success r => Success r
      | Failure err2 => Failure (List.append(err1, err2))

structure Ops = struct
   val (op >>=) = (op >>=)
   val (op <*>) = (op <*>)
   val (op <$>) = (op <$>)
   val (op <$?>) = (op <$?>)
   val (op <$$>) = (op <$$>)
   val (op <$$$>) = (op <$$$>)
   val (op <$$$$>) = (op <$$$$>)
   val (op <*) = (op <*)
   val (op *>) = (op *>)
   val (op <$) = (op <$)
   val (op <|>) = (op <|>)
end



fun failString (m, p : Location.t, s : (char * Location.t) Stream.t) =
   (m ^ " at " ^
      (Int.toString (#line p)) ^ ":" ^ (Int.toString (#column p)) ^
      "\n     Near: " ^ (String.implode (List.map(Stream.firstNSafe(s, 20), #1))))

fun fail m (s : State.t) = case Stream.force (s)
   of NONE => Failure []
    | SOME((_, p : Location.t), _) => Failure [failString (m, p, s)]

fun delay p = fn s => p () s

fun next (s : State.t)  = case Stream.force (s)
   of NONE => Failure ["Any character at end of file"]
    | SOME((h, _), r) => Success (h, r)

fun satExpects(t, p, m) s =
   case t s of
       Success (a, s') =>
         (if p a then Success (a, s') else fail m s)
     | Failure err => Failure err
     | FailCut err => FailCut err

fun sat(t, p) s = satExpects(t, p, "Satisfying") s
fun nextSat p s = case Stream.force s
   of NONE => Failure ["Any character at end of file"]
    | SOME((h, _), r) => (case p h of
         false => Failure ["Satisfying character"]
       | true => Success (h, r))

fun peek p (s : State.t) =
   case p s of Success (h', _) => Success (h', s)
             | err => err

fun failing p s =
   case p s
      of Success _ => fail "failure" s
       | _ => Success ((), s)

fun notFollowedBy(p, c) =
   p <* failing c

fun any'([]) s = Failure []
  | any'(p::ps) s =
       case p s of
          Success (a, s) => Success (a, s)
        | Failure m => (case any'(ps) s
           of Failure m2 => Failure (List.append(m, m2))
            | succ => succ)
        | FailCut m => FailCut m
fun 'a any ps = uncut (any' ps)


fun 'a many' (t : 'a t) s = case ((op ::) <$$> (t, fn s' => many' t s')) s of
    Success x => Success x
  | Failure y => pure [] s
  | FailCut z => FailCut z
fun 'a many t = uncut (many' t)
fun 'a many1 (t : 'a t) = uncut ((op ::) <$$> (t, many' t))

fun manyFailing(p, f) = many (failing f *> p)
fun manyCharsFailing f = many (failing f *> next)

fun sepBy1(t, sep) = uncut ((op ::) <$$> (t, many' (sep *> t)))
fun sepBy(t, sep) = uncut ((op ::) <$$> (t, many' (sep *> t)) <|> pure [])

fun optional t = SOME <$> t <|> pure NONE

fun char c s = case Stream.force (s)
   of NONE => Failure [String.fromChar c ^ " at end of file"]
    | SOME((h, _), r) =>
         if h = c
            then Success (h, r)
            else fail (String.fromChar c) s


fun each([]) = pure []
  | each(p::ps) = (curry (op ::)) <$> p <*> (each ps)


fun matchList s1 l2 = case (Stream.force s1, l2)
   of (_, []) => Success ((), s1)
    | (NONE, (_::_)) => Failure []
    | (SOME ((h, _), r), (x :: xs)) => if h = x then matchList r xs else Failure []
fun str str s = case matchList (s) (String.explode str)
   of Success ((), r) => Success (str, r)
    | _ => fail str s

fun location (s : State.t) = case Stream.force s of
       NONE => Failure ["any character end of file location"]
     | SOME((h, n), r) => Success (n, s)

fun toReader (p : 'a t) (s : State.t) : ('a * State.t) option =
   case p s of
      Success (a, s') => SOME (a, s')
    | _ => NONE

fun fromReader (r : State.t -> ('a * State.t) option) (s : State.t) =
   case r s of
      SOME (b, s') =>
         Success (b, s')
    | NONE => fail "fromReader" s

fun fromScan scan = fromReader (scan (toReader next))

val int = fromScan (Function.curry Int.scan StringCvt.DEC)

fun compose (p1 : char list t, p2 : 'a t) (s : State.t) =
   let
      (* easiest way to escape here *)
      exception ComposeFail of string list
      fun makeStr s' () = case Stream.force s' of
         NONE => Stream.empty ()
       | SOME ((_, pos), r) =>
            (case p1 s' of
                Success (b, r) => (case b of
                    (* equivalent, but avoids the jumping from append of fromList *)
                    c::[] => Stream.cons((c, pos), Stream.delay (makeStr r))
                  | _  => Stream.append
                        (indexStream(pos, Stream.fromList b),
                         Stream.delay (makeStr r)))
              | Failure m => raise ComposeFail m
              | FailCut m => raise ComposeFail m)
   in
      p2 (makeStr (s) () ) handle ComposeFail m => Failure m end

val space = nextSat Char.isSpace
val spaces = many space


(* The following parsers always (and only) consume spaces before
 * performing a `char` or `str`.
 *)

(* parse SML-style keyword (not followed by alphanum id character) *)
fun kw s =
   spaces *> str s *>
   failing (nextSat (fn c => Char.isAlphaNum c orelse c = #"_" orelse c = #"'"))
(* parse SML-style symbol (not followed by symbolic id character) *)
fun sym s =
   spaces *> str s *>
   failing (nextSat (fn c => String.contains ("!%&$#+-/:<=>?@\\~`^|*", c)))

(* parse `Bool.layout` *)
val bool: bool t =
   (true <$ kw "true") <|> (false <$ kw "false")

(* parse `Option.layout` *)
fun option (p: 'a t): 'a option t =
   (SOME <$> (kw "Some" *> p)) <|> (NONE <$ kw "None")

local
   fun between (l, p: 'a t, r): 'a t =
      spaces *> char l *> p <* spaces <* char r
in
   fun paren p = between (#"(", p, #")")
   fun cbrack p = between (#"{", p, #"}")
   fun sbrack p = between (#"[", p, #"]")
end
(* parse `List.layout` (not, `Layout.list`) *)
fun list (p: 'a t): 'a list t =
   sbrack (sepBy (p, spaces *> char #","))
fun listOpt (p: 'a t): 'a list t =
   list p <|> pure []
(* parse `Vector.layout` (not, `Layout.vector`) *)
fun vector (p: 'a t): 'a vector t =
   Vector.fromList <$> paren (sepBy (p, spaces *> char #","))
fun vectorOpt (p: 'a t): 'a vector t =
   vector p <|> pure (Vector.new0 ())

local
   fun field (s, p) = kw s *> sym "=" *> p
in
   (* parse first field of a record (not preceded by `,`) *)
   val ffield = field
   (* parse next field of a record (preceded by `,`) *)
   fun nfield (s, p) = spaces *> char #"," *> field (s, p)
end

local
   fun finiComment n () =
      any
      [str "(*" *> delay (finiComment (n + 1)),
       str "*)" *> (if n = 1 then pure [Char.space] else delay (finiComment (n - 1))),
       next *> delay (finiComment n)]

   val skipComments =
      any
      [str "(*" *> finiComment 1 (),
       (fn cs =>
        Char.dquote ::
        (List.foldr
         (cs, [Char.dquote], fn (c, cs) =>
          String.explode (Char.escapeSML c) @ cs))) <$>
       (char Char.dquote *>
        many (fromScan Char.scan) <*
        char Char.dquote),
       each [next]]
in
   val skipCommentsML = skipComments
end

end
